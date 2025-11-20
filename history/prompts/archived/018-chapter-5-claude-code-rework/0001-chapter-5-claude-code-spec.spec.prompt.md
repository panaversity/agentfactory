---
id: 0001
title: Chapter 5 Claude Code Specification
stage: spec
date: 2025-01-13T18:30:00Z
surface: slash-command
model: claude-sonnet-4-5
feature: 018-chapter-5-claude-code-rework
branch: 018-chapter-5-claude-code-rework
user: orchestrator
command: /sp.specify Chapter 5 rework - "How It All Started: The Claude Code Phenomenon"
labels: [spec, chapter-rework, claude-code, part-2, evals-first, ai-usage-strategy]
links:
  spec: specs/018-chapter-5-claude-code-rework/spec.md
  ticket: https://github.com/panaversity/ai-native-software-development/issues/132
  adr: null
  pr: null
files:
  - path: specs/018-chapter-5-claude-code-rework/spec.md
    status: created
    lines: 331
  - path: specs/018-chapter-5-claude-code-rework/checklists/requirements.md
    status: created
    lines: 140
tests:
  - type: spec_validation
    status: passed
    coverage: 100%
    notes: All checklist items passed validation
---

## Prompt

User request: "We have to rework on chapter 5 claude code - it is missing a lot of work and many things are hallucinated or not explained. Now we have a sandbox so you can setup claude there and test everything from installation to helloworld etc. We can use context7 mcp tools and web search to fully understand claude code and rework on this chapter to make it well aligned with our book vision, preface and part1 students have just completed before this chapter"

Context provided:
- Constitution v3.1.3 (AI-native development methodology, Graduated Teaching Pattern, Three Roles Framework)
- Chapter Index (Chapter 5 in Part 2, A2 tier audience, follows Chapters 1-4)
- Existing Chapter 5 content (origin story, installation, subagents, skills, MCP lessons)
- Official Claude Code docs (from code.claude.com via WebFetch + Context7 MCP /anthropics/claude-code library)
- Chapter 12 UV review lessons (anti-over-engineering patterns: don't use AI for simple commands)
- User confirmation: AIDD philosophy (co-learning, co-developing with coding agents, subagents, skills as reusable intelligence)

Intelligence Object derived:
```json
{
  "task_type": "book_chapter_rework",
  "chapter_number": 5,
  "part_number": 2,
  "audience_tier": "aspiring",
  "complexity_level": "A2",
  "prerequisites": ["Chapter 1-4"],
  "domain_skills": ["learning-objectives", "concept-scaffolding", "code-example-generator", "exercise-designer", "technical-clarity", "book-scaffolding", "ai-collaborate-teaching"],
  "teaching_pattern": "direct_commands_for_installation",
  "ai_usage_strategy": "Direct commands for installation (curl/npm ~30 sec). AI for understanding concepts, troubleshooting, strategic decisions.",
  "cognitive_load_limit": 7,
  "current_issues": ["Missing comprehensive coverage", "Hallucinated/outdated info", "Not aligned with book vision/Part 1", "No sandbox validation", "Missing AI usage strategy clarity"]
}
```

## Response snapshot

**Phase 0: Intelligent Context Discovery** ✅ COMPLETED
- Read constitution.md (v3.1.3, 1580 lines)
- Read chapter-index.md (83 chapters, Chapter 5 in Part 2)
- Read existing Chapter 5 content (README + 5 lesson files)
- WebSearch: Claude Code CLI official documentation 2025
- Context7 MCP: Retrieved /anthropics/claude-code library (37 code snippets, 8.8 trust score)
- WebFetch: code.claude.com/docs/en/overview (comprehensive overview)

**Key Intelligence Gathered**:
1. **Official Claude Code Capabilities** (fact-checked):
   - Installation: curl script (macOS/Linux), PowerShell (Windows), Homebrew, NPM
   - Authentication: Interactive login after `claude` command
   - Subagents: Plan mode, Explore agent (Haiku 4.5 powered)
   - Skills: Specialized capabilities discovered automatically
   - MCP Integration: Connect to Google Drive, Jira, custom tools
   - Direct actions: Edit files, execute commands, create commits

2. **Constitutional Alignment Requirements**:
   - Principle 13: Graduated Teaching (Book teaches → AI companion → AI orchestration)
   - Principle 18: Three Roles (AI as Teacher/Student/Co-Worker)
   - Core Philosophy #1: AI Spectrum (Assisted → Driven → Native)
   - Cognitive Load: Max 7 concepts/section, 2 options (A2 tier)

3. **Anti-Over-Engineering Lessons** (from Chapter 12 UV review):
   - ❌ DON'T: Use AI for simple commands (`curl | bash`, `npm install`)
   - ❌ DON'T: Inflate durations (installation ~2-5 min, not 45+ min)
   - ❌ DON'T: Create 8+ verbose "Try with AI" prompts
   - ✅ DO: Direct commands for installation, AI for troubleshooting/understanding
   - ✅ DO: Use Chapter 1 clean format (3-4 focused prompts)

**Phase 1: Specification Creation** ✅ COMPLETED

Created comprehensive 331-line specification with:

**5 User Stories** (prioritized):
1. (P1) Understand Claude Code's Paradigm Shift - foundational mental model
2. (P1) Install and Authenticate Successfully - blocker for all subsequent work
3. (P2) Understand and Use Subagents - orchestration concept introduction
4. (P3) Understand Skills and MCP Integration - extensibility awareness
5. (P2) Practice with "Hello World" Workflows - confidence building

**26 Functional Requirements** (organized by category):
- Content Accuracy (FR-001 to FR-004): Sandbox testing, fact-checking, zero hallucinations, realistic durations
- Constitutional Alignment (FR-005 to FR-009): Book vision, Graduated Teaching, Three Roles, AI Spectrum, cognitive load
- AI Usage Strategy (FR-010 to FR-014): Direct commands for installation, no over-engineering, Chapter 1 format
- Pedagogical Structure (FR-015 to FR-019): 5 lessons, learning objectives, troubleshooting, comparison table
- Part 1 Integration (FR-020 to FR-022): Assumes Chapters 1-4, no repetition, explicit bridging
- Technical Validation (FR-023 to FR-026): Multi-platform testing, authentication verification, sandbox execution

**16 Success Criteria + 5 Evals**:
- Student Learning: 95% install success, 90% understand paradigm, 100% complete Hello World, 85% identify when to use AI
- Technical Accuracy: 0 hallucinations, 100% platform compatibility, realistic durations (±20% margin)
- Constitutional Compliance: Graduated Teaching demonstrated, Three Roles applied, cognitive load respected
- AI Strategy Validation: 0 over-engineering instances, clean format, strategic understanding
- Integration: Part 1 bridge, no contradictions, Part 2 positioning

**5 Evals with Business Goals**:
1. Installation Success Rate (95% target) → Minimize support burden
2. Conceptual Understanding (90% target) → Strategic tool selection mental model
3. Hands-On Completion (100% target) → Build confidence
4. AI Usage Strategy (85% target) → Prevent over-engineering pattern
5. Technical Accuracy (0 errors target) → Build trust and credibility

**Phase 1: Validation** ✅ ALL ITEMS PASSED

Created requirements.md checklist (140 lines) and validated:
- Content Quality: No implementation details, focused on user value, stakeholder language, all sections complete
- Requirement Completeness: No clarifications needed, testable, measurable, technology-agnostic, scenarios defined, edge cases identified, scope bounded
- Feature Readiness: Clear acceptance criteria, primary flows covered, measurable outcomes, no implementation leaks
- Constitutional Alignment: Principles 13, 18, Core Philosophy #1, cognitive load limits, anti-over-engineering enforced
- Additional Context: User feedback addressed (Issue #132), AIDD philosophy integrated, official docs referenced

## Outcome

- ✅ Impact: Created comprehensive, fact-checked specification for Chapter 5 rework with evals-first approach, constitutional alignment, and anti-over-engineering safeguards built-in
- 🧪 Tests: Spec validation checklist 100% passed (all items green)
- 📁 Files:
  - `specs/018-chapter-5-claude-code-rework/spec.md` (331 lines)
  - `specs/018-chapter-5-claude-code-rework/checklists/requirements.md` (140 lines)
- 🔁 Next prompts: `/sp.plan` (create lesson-by-lesson implementation plan with AI usage strategy mapped)
- 🧠 Reflection: Specification demonstrates strong vertical intelligence application (constitution → chapter-index → existing content → official docs → anti-patterns). Evals-first approach with business goals explicit. AI usage strategy prevents Chapter 12-style over-engineering. Ready for planning phase.

## Evaluation notes (flywheel)

- Failure modes observed: None - all validation items passed on first iteration
- Graders run and results (PASS/FAIL):
  - ✅ PASS: Spec quality checklist (100% items passed)
  - ✅ PASS: Constitutional alignment check (Principles 13, 18, Core Philosophy #1 enforced)
  - ✅ PASS: Anti-over-engineering check (FR-010 to FR-014, SC-011 to SC-013)
  - ✅ PASS: Evals-first validation (5 evals with business goals before implementation details)
- Prompt variant (if applicable): N/A - standard /sp.specify workflow
- Next experiment (smallest change to try): Continue to /sp.plan phase to create lesson-by-lesson implementation plan with teaching strategy mapped (Tier 1/2/3 breakdown per lesson)
