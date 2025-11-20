---
id: 003
title: Plan colearning skills architecture with Anthropic resources
stage: plan
date: 2025-10-28
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-colearning-skills
branch: 001-colearning-skills
user: user
command: /sp.plan
labels: ["plan", "architecture", "skills", "anthropic", "pedagogy", "research"]
links:
  spec: specs/001-colearning-skills/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-colearning-skills/plan.md
  - specs/001-colearning-skills/research.md
  - specs/001-colearning-skills/data-model.md
  - specs/001-colearning-skills/quickstart.md
  - specs/001-colearning-skills/contracts/skill-metadata.schema.yml
  - specs/001-colearning-skills/contracts/learning-objective.schema.yml
  - specs/001-colearning-skills/contracts/code-example.schema.yml
tests:
  - Constitution check: All 11 principles reviewed, all passed or deferred appropriately
  - Research validation: 7 decision areas researched using Anthropic official docs + pedagogy research
  - Data model completeness: 6 skill entities designed with validation rules
  - Schema validation: 3 YAML schemas created for key entities
---

## Prompt

/sp.plan let's carefully plan and instead of assumption from intelligence use the antropic official resources and we have to build the effective skills thtat can add effective measurable outcomes to plan content as educational curriculum strategist

## Response snapshot

Executed comprehensive `/sp.plan` workflow focusing on Anthropic's official skill documentation and evidence-based pedagogical research.

**Phase 0: Research (Complete)**

Researched 7 critical areas using official sources:

1. **Claude Code Skills Architecture**: Confirmed progressive disclosure (3 layers: metadata → instructions → resources) from Anthropic docs and engineering blog (October 2025)

2. **SKILL.md Requirements**: Documented frontmatter format (name, description, allowed-tools), activation best practices, max lengths

3. **File Organization**: Established type-based directory structure (reference/, templates/, scripts/) within each skill directory

4. **Pedagogical Foundations**:
   - Bloom's Taxonomy for programming (ACL 2025 research: LLMs underserve Metacognitive/Create/Evaluate levels)
   - Cognitive Load Theory (CLT) principles: working memory limit (7±2 chunks), scaffolding strategies
   - Evidence-based strategies: worked examples, spaced repetition, interleaving

5. **Sandbox Execution**: Designed Python subprocess approach with security constraints (timeout, isolated temp dir, no network access) per user's choice in clarification #3

6. **Skill Conflict Resolution**: Mutually exclusive descriptions + sequential activation per clarification #5

7. **Output Format Flexibility**: No prescribed format per clarification #4; each skill uses optimal structure

**Key Research Sources**:
- Anthropic Claude Code Skills docs: https://docs.claude.com/en/history/claude-code/skills
- Anthropic engineering blog: https://www.anthropic.com/engineering/equipping-agents-for-the-real-world-with-agent-skills
- ACL 2025: "LLMs meet Bloom's Taxonomy"
- ResearchGate: "Managing Cognitive Load in Introductory Programming Courses"
- British Journal of Educational Technology 2025: "GenAI-enabled coding hints and cognitive load"

**Phase 1: Design (Complete)**

Created comprehensive architectural design:

1. **Data Models** (data-model.md):
   - 3-layer skill structure (metadata, instructions, resources)
   - 6 skill-specific output entities with validation rules
   - File system structure for `.claude/skills/`
   - Data flow diagrams (single skill, sequential multi-skill)
   - Error handling strategy (hard failure mode)
   - Performance targets aligned with success criteria

2. **Contracts** (YAML schemas):
   - `skill-metadata.schema.yml`: Validates SKILL.md frontmatter
   - `learning-objective.schema.yml`: Validates learning-objectives output
   - `code-example.schema.yml`: Validates code-example-generator output with all compliance checks

3. **Quickstart Guide** (quickstart.md):
   - Installation instructions
   - Usage examples for all 6 skills
   - Activation patterns explanation
   - Troubleshooting guidance
   - Best practices for educators

4. **Comprehensive Plan** (plan.md):
   - Constitution check: All 11 principles reviewed ✅
   - Technical context fully specified
   - 6 architecture decisions documented with rationales
   - Risk analysis with 5 identified risks + mitigations
   - Success criteria alignment table (SC-001 to SC-015)
   - Open questions for Phase 2 (task decomposition)

**Architecture Decisions Made**:

1. Progressive disclosure with 3-layer filesystem loading
2. Subprocess sandbox with 5s timeout and security constraints
3. Mutually exclusive descriptions + sequential activation for multi-skill workflows
4. Flexible output formats (skill-appropriate structures)
5. Evidence-based pedagogy (Bloom's + CLT + scaffolding)
6. Type-based file organization (reference/, templates/, scripts/)

**Artifacts Created**: 7 files
- plan.md (comprehensive architecture)
- research.md (consolidated findings)
- data-model.md (entity structures)
- quickstart.md (user guide)
- 3 YAML schemas (contracts/)

## Outcome

- ✅ Impact: Complete architectural plan based on Anthropic official resources and pedagogical research. All unknowns from Technical Context resolved. 6 core skills fully designed with data models, validation rules, and implementation strategy.
- 🧪 Tests: Constitution check passed (all 11 principles), research validated against official sources, data models include validation rules and constraints, schemas provide contract specifications
- 📁 Files: Created 7 new files across specs/001-colearning-skills/
- 🔁 Next prompts: Run /sp.tasks to generate dependency-ordered task list for implementation
- 🧠 Reflection: User's guidance to "use Anthropic official resources" was critical - ensured architecture follows actual documented patterns rather than assumptions. Progressive disclosure is THE key pattern. Pedagogy research provides measurable foundation (not opinions). Sandbox execution (user's choice) adds implementation complexity but ensures code quality.

## Evaluation notes (flywheel)

- Failure modes observed: Initial script failures due to Windows line endings (worked around by manual path resolution); no architectural failures
- Graders run and results (PASS/FAIL): PASS - Constitution check completed, all research sourced from official docs, data models complete with validation, plan addresses all spec requirements
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): During /sp.tasks, prioritize sandbox script implementation early to validate feasibility and identify any platform-specific issues before building dependent skills
