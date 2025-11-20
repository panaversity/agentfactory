---
id: 0002
title: "Clarify Part 5 Pedagogical Design"
stage: misc
date: 2025-11-01
surface: agent
model: claude-haiku-4-5-20251001
feature: 008-part-5-sdd
branch: 008-part-5-sdd
user: mjs
command: /sp.clarify
labels: [part5, pedagogical-design, ai-collaboration, specifyplus, capstone-project]
links:
  spec: specs/008-part-5-sdd/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/008-part-5-sdd/spec.md (updated with clarifications)
tests:
  - All 5 clarification questions answered
  - Spec sections updated with detailed pedagogical models
---

## Prompt

```
Now let's ensure we are planning correctly the specification for this part. We have @context/14_chap30_specs/ and:

1. Spec Driven Development: https://github.com/panaversity/spec-kit-plus/blob/main/spec-driven.md
2. SpecifyPlus QuickStart Introduction: https://github.com/panaversity/spec-kit-plus/tree/main/docs-plus/05_quickstart
3. Core Concepts: https://github.com/panaversity/spec-kit-plus/tree/main/docs-plus/06_core_commands
4. Applied SpecifyPlus Agentic project (Clone a Funded AI Startup): https://github.com/panaversity/spec-kit-plus/tree/main/docs-plus/07_professional_assignment

The book will have 3 chapters and will create and give users the framework for AI Collaborative development like we discussed in @book-source/docs/preface-agent-native.mdx

We need to understand: what will be covered in each chapter? And we're teaching to Collaborate with AI Companion (Gemini CLI/Claude Code or Others) to learn, build and ship under SpecifyPlus Framework. We're challenging traditional education so we need to carefully plan where user and their AI Companion are actually collaborating to learn.
```

## Response snapshot

Conducted 5-question clarification session to design Part 5's pedagogical architecture:

**Clarifications Resolved**:

1. **Chapter 25 (Fundamentals)**: Concept-first + Spec-First Discovery
   - Students learn SDD concepts, then engage in AI collaboration discovery
   - Write rough specs → AI generates implementations → Compare outputs → Refine together

2. **Chapter 26 (SpecifyPlus Hands-On)**: Real project specs + SpecifyPlus tools
   - Students install SpecifyPlus: `uvx specifyplus init <projectname>`
   - Select AI companion (Claude Code, Gemini CLI) and shell environment
   - Progressive projects: Python calculator (simple SDD loop) → Grading system (real project)

3. **Chapter 27 (Team Collaboration)**: Multi-agent parallel development
   - Students set up 2 SpecifyPlus instances with 2 AI companions
   - Feature 1: Grading engine (AI Companion A) | Feature 2: Feedback generation (AI Companion B)
   - Learn how specifications enable parallel, async team development

4. **Learning Progression**: Interleaved theory + practice
   - Ch 25: Concepts → Ch 26: SpecifyPlus + hands-on → Ch 27: Full capstone with team simulation
   - Theory and practice run in parallel for maximum engagement

5. **Capstone Project**: Real-world grading system (YC-validated market)
   - "Assignments Grading and Feedback Agentic System"
   - Real startups solving this (Gradewiz, Edexia, Frizzle, Mimir)
   - Students build something that matters

**Key Shift**: Part 5 teaches SDD *through* SpecifyPlus as the learning environment, not about SpecifyPlus as a tool. Students think, build, and ship under an opinionated framework.

## Outcome

- ✅ Impact: Transformed Part 5 spec from generic "SDD methodology" to specific **AI-collaborative pedagogical architecture** grounded in real SpecifyPlus usage and real-world problem
- 🧪 Tests: All 5 questions answered; spec sections completely rewritten with concrete learning models
- 📁 Files: specs/008-part-5-sdd/spec.md updated with detailed pedagogical models for each chapter
- 🔁 Next prompts: `/sp.plan` ready (no further clarifications needed)
- 🧠 Reflection: User emphasized "challenging traditional education" through co-learning. This reframing changed the entire spec: from theoretical methodology chapter → actual SpecifyPlus hands-on learning environment. Students aren't learning SDD in abstract; they're building real projects with real AI companions under a real framework that mirrors professional workflows.

## Evaluation notes (flywheel)

- Failure modes observed: None; user provided clear, specific context and preferences
- Graders run and results (PASS/FAIL): Spec quality checklist ready for update; all pedagogical patterns now explicit
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): Run `/sp.plan` to generate detailed chapter plans with specific lesson sequences, projects, and exercises that operationalize the pedagogical models

---

## Subagents & Skills Used

**Subagents Invoked**: None (main Claude orchestrated clarification workflow)

**Skills Applied**:
- `concept-scaffolding`: Mapped SpecifyPlus resources to chapter progression
- `learning-objectives`: Clarified learning outcomes for each chapter's AI collaboration pattern
- `book-scaffolding`: Ensured chapters build progressively and connect to adjacent parts
- `content-evaluation-framework`: Validated pedagogical decisions against complexity tier standards

---

## Essential Learnings for Vertical Intelligence System

**Learning 5: Pedagogical Models Must Be Explicitly Operationalized**
- Generic "Chapter 26 teaches spec-writing" required 5 clarification questions to become concrete
- Insight: Educational specs must explicitly describe **AI collaboration cycles**, not just topics
- System implication: Educational spec template needs mandatory "AI Collaboration Pattern" section
- Actionable: For future parts, specify: AI tool(s) → Student action → AI response → Validation

**Learning 6: Complexity Tier Enforcement Prevents Rework**
- Advanced tier (10+ concepts) wasn't enforced in initial spec; clarifications made it explicit
- Insight: Complexity tier changes pedagogy fundamentally, not just difficulty
- System implication: Create complexity tier validator checking: cognitive load, student responsibility, scaffolding approach
- Actionable: Each part spec must state tier and 3-4 tier-specific standards

**Learning 7: Three-Chapter Methodology Arc is Optimal**
- User-guided design evolved naturally into: Fundamentals (WHY) → Hands-On (HOW) → Mastery (SCALE)
- Insight: This pattern applies beyond SDD to any methodology teaching
- System implication: Use as template for Parts 6, 10, 11, 13 (agents, deployment, databases, stateful systems)
- Actionable: Create 3-chapter arc template for methodology parts

**Learning 8: Mini-Projects as Progressive Scaffolding**
- Python calculator (safe, bounded) → Real grading system (domain, stakes) → Capstone (full team) worked better than single project
- Insight: Bounded projects **master methodology in safe space** before real-world application
- System implication: Always design 2-3 progressively complex projects for methodology teaching
- Actionable: Create project progression template (bounded → real-world → capstone with team)

**Learning 9: Multi-Agent Coordination Teaches Team Concepts**
- 2 parallel AI companions on different features teaches specification-based collaboration
- Insight: Students experience why clear specs matter when watching independent AI agents stay coordinated
- System implication: For scaling/team parts (25-27, 30, 42, 47, 49), include multi-agent simulation
- Actionable: Design feature decomposition + parallel AI pattern for Parts 6-13

**Learning 10: Paradigm Shift Questions Unlock Better Designs**
- "Challenging traditional education" reframed entire pedagogical approach
- Insight: Users often have deeper intentions that transform design when articulated
- System implication: Add explicit questions: "What paradigm are we moving away from?" and "What's the philosophical shift?"
- Actionable: Include "paradigm shift" section in clarification workflow
