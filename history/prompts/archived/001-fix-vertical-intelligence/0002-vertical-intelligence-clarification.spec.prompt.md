---
id: 0002
title: vertical-intelligence-clarification
stage: spec
date: 2025-11-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-fix-vertical-intelligence
branch: 001-fix-vertical-intelligence
user: mjs
command: /sp.clarify
labels: ["validation", "evaluation-scenarios", "ai-orchestrator", "metadata", "data-driven"]
links:
  spec: specs/001-fix-vertical-intelligence/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-fix-vertical-intelligence/spec.md (updated with clarifications)
tests:
  - Validated clarifications against spec coverage taxonomy
---

## Prompt

Now you have our initalizl diescussion and your own audit context along with the external audit report and spec. Let's ensure we know the core problems and how to fix them. Before fixing we need to know the Evalution Scenarios like what does the success looks like

## Response snapshot

Ran /sp.clarify to identify ambiguities and clarify evaluation scenarios. User provided two critical insights:

1. **Book authoring is AI-orchestrator collaboration**: Domain experts (authors) work with AI orchestrators (Claude, subagents) in iterative workflow: specification → planning → tasks → implementation → validation. Each phase involves AI presenting outlines and clarifying with human.

2. **Validation requires AI intelligence + human judgment**: Cannot lose intelligence by making validation purely manual. AI should present evidence and analysis; human reviews and confirms. Combined approach: incremental validation after each layer + test-driven end-to-end validation with generated test chapter.

3. **Metadata tracking requirement**: All generated content must include comments showing: source spec used, tool/subagent that created it, git author. Enables traceability, auditability, debugging, and quality metrics.

**Key Clarifications Added to Spec**:

- **Validation Approach section**: Detailed hybrid AI-orchestrator model with:
  - Incremental layer validation (AI analyzes → human checkpoints → audit trail)
  - Test-driven end-to-end validation (generate test chapter → automated analysis → human review)
  - Data-driven metrics (terminology consistency score, structure match score, zero-correction rate, workflow success rate)

- **Success Validation Matrix**: Table mapping each success criterion to AI validation method, human confirmation approach, and measurable data metric

- **Content Generation Metadata Requirements**: Mandatory metadata comment format for all generated content with implementation guidance for subagents

## Outcome

- ✅ Impact: Spec now includes comprehensive validation approach matching AI-orchestrator collaboration model, with data-driven metrics to measure vertical intelligence quality
- 🧪 Tests: Added 4 quantitative metrics (all targeting 100%): terminology consistency, structure match, zero-correction rate, workflow success rate
- 📁 Files: spec.md updated with 3 new major sections (112 lines added)
- 🔁 Next prompts: `/sp.plan` to generate detailed implementation plan including validation infrastructure tasks
- 🧠 Reflection: User's insight about "losing intelligence with pure manual validation" was critical - led to hybrid approach where AI does intelligent analysis and human provides judgment, matching book authoring workflow

## Evaluation notes (flywheel)

- Failure modes observed: Initial question framing was too software-centric (automated vs manual). User reframed to match actual domain (book authoring with AI-orchestrator collaboration)
- Graders run and results (PASS/FAIL): Spec coverage analysis PASSED - 2 clarifications resolved "Partial" categories to "Clear"
- Prompt variant (if applicable): Standard /sp.clarify workflow adapted for book infrastructure project
- Next experiment (smallest change to try): When implementing validation, start with one layer (constitution) to validate the incremental validation workflow before applying to all layers
