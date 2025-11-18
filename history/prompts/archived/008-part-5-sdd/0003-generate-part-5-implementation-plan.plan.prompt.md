---
id: 0003
title: "Generate Part 5 Implementation Plan"
stage: plan
date: 2025-11-01
surface: agent
model: claude-haiku-4-5-20251001
feature: 008-part-5-sdd
branch: 008-part-5-sdd
user: mjs
command: /sp.plan
labels: [part5, planning, specifyplus, pedagogical-design, capstone-project, multi-agent]
links:
  spec: specs/008-part-5-sdd/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/008-part-5-sdd/plan.md (generated, 364 lines)
tests:
  - Plan structure validated (3 chapters + 18 lessons + 3 projects)
  - Learning objectives aligned with specification
  - Time estimates realistic (32-40 hours over 5-6 weeks)
  - AI collaboration patterns operationalized
---

## Prompt

```
Generate comprehensive implementation plan for Part 5: Spec-Kit Plus Methodology (Chapters 25-27).

Input: Specification from `/specs/008-part-5-sdd/spec.md` with pedagogical models:
- Ch 25: Concept-first + Spec-First Discovery (5 lessons)
- Ch 26: SpecifyPlus hands-on + 2 mini-projects (7 lessons + 2 projects)
- Ch 27: Team collaboration capstone (6 lessons + 1 capstone project)

Output: Detailed implementation plan with:
- Chapter breakdown (lessons, objectives, exercises)
- Mini-project progressions (Python calculator → Grading system)
- Capstone details (2 parallel AI companions, feature decomposition)
- Part 5 README structure
- Learning time estimates
- Connection to Parts 1-4 and Parts 6+

Key constraint: Operationalize pedagogical models into concrete lesson sequences ready for content-implementer implementation.
```

## Response snapshot

**Plan Generated**: 364-line comprehensive implementation plan covering:

**Chapter 25: SDD Fundamentals** (8-10 hours, 5 lessons)
- Lesson 1: What is SDD? (crisis case studies, market validation)
- Lesson 2: Cost of specs vs. code (35-50% rework without specs, 15-20% with specs)
- Lesson 3: Spec-First Discovery with AI (vague spec → refine → AI excels)
- Lesson 4: The SDD Loop (5-phase workflow: Spec → Plan → Tasks → Implement → Validate)
- Lesson 5: AI-native connection (why this matters for their career)
- AI Collaboration: ChatGPT web + reflection prompts (conceptual, no coding yet)

**Chapter 26: SpecifyPlus Hands-On** (12-15 hours, 7 lessons + 2 mini-projects)
- Lessons: Spec structure, SpecifyPlus setup, requirements, acceptance criteria, core commands, refinement, integration
- **Mini-Project 1 (Calculator)**: Full SDD cycle (spec → `/sp.plan` → `/sp.tasks` → implement → validate) in bounded scope
- **Mini-Project 2 (Grading System)**: YC-validated real-world problem introduction; foundation for Chapter 27
- Tools: Claude Code / Gemini CLI with hands-on SpecifyPlus commands

**Chapter 27: Team Collaboration & Capstone** (12-15 hours, 6 lessons + capstone)
- Lessons: Scaling (1→5→10+ person teams), feature decomposition, workflows, async coordination, integration patterns, SDD philosophy
- **Capstone**: "Assignments Grading & Feedback Agentic System" with 2 parallel AI companions
  - Part 1: Master spec + feature decomposition (2-3 hours)
  - Part 2: Parallel AI implementation (grading engine vs. feedback generation) (2-3 hours)
  - Part 3: Reflection, synthesis, professional commitments (1-2 hours)

**Total Part 5**: 18 lessons + 3 projects = 32-40 hours over 5-6 weeks

**Pedagogical Strengths**:
✅ Interleaved theory + practice (maximum engagement)
✅ Progressive complexity (understand → practice → master)
✅ Real-world relevance (YC-validated grading market)
✅ AI collaboration throughout (Spec-First Discovery → hands-on → multi-agent coordination)
✅ Advanced complexity tier maintained (10+ concepts, no artificial scaffolding)

## Outcome

- ✅ Impact: Enabled content-implementer subagent to implement Part 5 with clear, detailed, pedagogically sound lesson sequences
- 🧪 Tests: Plan structure complete (364 lines); all 3 chapters mapped to lessons with objectives; mini-projects and capstone specified
- 📁 Files: `/specs/008-part-5-sdd/plan.md` (364 lines, ready for lesson writing)
- 🔁 Next prompts: `/sp.tasks` (generate actionable task checklist) then `content-implementer` subagent (implement chapters)
- 🧠 Reflection: Plan operationalizes the pedagogical model from clarifications. The progression is clear: WHY (Ch 25) → HOW (Ch 26) → SCALE (Ch 27). Each chapter has concrete lessons, exercises, and hands-on projects. The capstone ties it all together with real-world relevance (grading system is proven market problem with YC backing).

## Evaluation notes (flywheel)

- Failure modes observed: None; specification and clarifications provided clear direction
- Graders run and results (PASS/FAIL): Plan aligns with spec pedagogical models; Advanced complexity tier maintained; learning objectives follow Bloom's taxonomy
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): Generate tasks.md using `/sp.tasks` to break plan into atomic, testable work items for content-implementer implementation

---

## Subagents & Skills Used

**Subagents Invoked**:
- `chapter-planner`: Generated detailed 364-line implementation plan with lesson sequences, objectives, projects, and capstone details

**Skills Applied** (by chapter-planner):
- `learning-objectives`: Defined measurable outcomes for each lesson aligned with Bloom's taxonomy
- `concept-scaffolding`: Structured progressive complexity (Understand Ch 25 → Practice Ch 26 → Master Ch 27)
- `book-scaffolding`: Positioned chapters relative to prerequisites (Parts 1-4) and enablement (Parts 6+)
- `exercise-designer`: Designed AI collaboration exercises (Spec-First Discovery, SpecifyPlus hands-on, multi-agent capstone)
- `code-example-generator`: Specified code examples with purpose, complexity, and validation patterns
- `assessment-builder`: Defined learning outcomes testable through projects and capstone
- `technical-clarity`: Ensured plan terminology consistent with SpecifyPlus framework and AI-native concepts
- `ai-collaborate-teaching`: Applied co-learning pedagogy throughout (students and AI working together in SpecifyPlus environment)

---

## Essential Learnings for Vertical Intelligence System

**Learning 11: Chapter-Planner Subagent Effectiveness**
- Chapter-planner successfully operationalized pedagogical models into concrete lesson sequences
- Insight: Subagents with clear spec + clarifications can generate implementation-ready plans without human intervention
- System implication: Chapter-planner is effective for methodology parts; works well when given:
  - Clear pedagogical models (Concept-first, hands-on, capstone)
  - Real-world context (YC companies, market validation)
  - Progression framework (mini-projects, capstone)
- Actionable: Use chapter-planner for all methodology parts (5, 6, 10, 11, 13); ensure specs include pedagogical models

**Learning 12: Lesson Sequencing Patterns for Methodology**
- Plan reveals natural lesson sequencing: Conceptual foundations → Hands-on basics → Progressive complexity → Real-world application
- Insight: Methodology lessons benefit from this specific progression more than technical topics
- System implication: Don't vary lesson order arbitrarily; use this pattern for methodology parts
- Actionable: Create lesson sequencing template for methodology chapters

**Learning 13: Mini-Project + Capstone Architecture**
- Plan demonstrates effective 3-project progression:
  1. Mini-Project 1 (bounded): Master methodology in safe space
  2. Mini-Project 2 (real-domain): Apply to real problem
  3. Capstone (full-scale): Master at team scale
- Insight: This progression maximizes learning while minimizing cognitive overload
- System implication: Standardize this pattern for Parts requiring complex skill building
- Actionable: Create project architecture template with 3-tier progression

**Learning 14: Real-World Market Context Strengthens Pedagogy**
- Plan emphasizes YC-validated grading market (Gradewiz, Edexia, Frizzle, Mimir) throughout capstone
- Insight: When students understand real market exists, intrinsic motivation increases dramatically
- System implication: Every capstone project should have researched market validation
- Actionable: Maintain database of YC companies by domain; reference in capstone specifications

**Learning 15: Time Estimates Must Be Realistic**
- Plan allocates 32-40 hours over 5-6 weeks for Part 5 (3 chapters + 3 projects)
- Insight: This allows 6-8 hours per chapter including projects; realistic for advanced learners
- System implication: Don't compress methodology teaching; students need time to practice and internalize
- Actionable: Use realistic time estimates to govern scope (better to go deep on 3 chapters than shallow on 5)

**Learning 16: Lesson Complexity Follows Predictable Pattern**
- Chapter 25 (concepts): 5 lessons × 1.5-2 hours = 8-10 hours
- Chapter 26 (hands-on): 7 lessons + 2 projects × 1.5 hours/lesson + 3-4 hours/project = 12-15 hours
- Chapter 27 (capstone): 6 lessons + 1 capstone × 1.5 hours/lesson + 5-7 hours/capstone = 12-15 hours
- Insight: Lesson density increases as students progress; projects take non-linear time
- System implication: Create lesson complexity estimator based on type (concept, hands-on, capstone)
- Actionable: Use time estimates for other parts; adjust based on student profile

**Learning 17: Multi-Agent Simulation Requires Careful Design**
- Capstone design (2 AI companions on parallel features) teaches team concepts without complexity
- Insight: Multi-agent coordination through specifications is powerful pedagogy when well-designed
- System implication: Include multi-agent patterns in parts 6-13 (agents, deployment, databases)
- Actionable: Create multi-agent simulation design patterns for scaling/team concepts

**Learning 18: Connection to Adjacent Parts Strengthens Architecture**
- Plan explicitly connects to:
  - Prerequisite knowledge (Parts 1-4: mindset, tools, prompting, Python)
  - Future application (Parts 6+: agents need specs, deployment needs specs, databases need specs)
- Insight: When chapters explicitly show "why this enables later work," motivation increases
- System implication: Always design chapters with forward/backward connections
- Actionable: Create connection mapping for all 13 parts showing dependency graph

**Learning 19: Pedagogical Model Operationalization Process**
- Process: Spec (pedagogical model) → Clarify (make explicit) → Plan (concrete lessons) → Tasks (atomic work)
- Insight: This SpecKit Plus workflow works perfectly for educational content design
- System implication: Educational content should always follow this workflow; don't skip phases
- Actionable: Enforce spec → clarify → plan → tasks workflow for all book chapters

---

## System-Level Recommendations

### **For Vertical Intelligence System**

1. **Create Educational Content Specialization Module**
   - Understand pedagogical patterns (scaffolding, cognitive load, assessment)
   - Recognize real-world validation (market demand, company examples)
   - Apply complexity tiers systematically
   - Design co-learning environments intentionally

2. **Build Pedagogical Model Library**
   - Spec-First Discovery (concept learning with AI)
   - Environment-Based Pedagogy (framework-as-learning-environment)
   - Progressive Projects (bounded → real-world → capstone)
   - Multi-Agent Simulation (team concepts without actual teams)
   - Three-Chapter Arc (WHY → HOW → SCALE)

3. **Develop Domain-Specific Knowledge**
   - **Educational Content**: Complexity tiers, pedagogical patterns, co-learning cycles
   - **SpecifyPlus Integration**: How to teach SDD through framework, not about framework
   - **Market Validation**: YC companies by domain for capstone projects
   - **Team Patterns**: How specifications enable async collaboration

4. **Create Planning Templates**
   - 3-chapter methodology arc template
   - Mini-project + capstone progression template
   - Multi-agent simulation design pattern
   - Lesson complexity estimator
   - Time allocation framework (5-6 weeks per part recommended)
