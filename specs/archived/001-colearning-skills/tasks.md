# Tasks: Colearning Skills for Python Book Agent

**Input**: Design documents from `/specs/001-colearning-skills/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md, contracts/

**Tests**: Integration tests included to validate skill activation and output correctness

**Organization**: Tasks are grouped by user story (8 skills = 8 user stories) to enable independent implementation and testing of each skill.
- **6 Core Skills (P1)**: learning-objectives, concept-scaffolding, code-example-generator, exercise-designer, technical-clarity, assessment-builder
- **2 Enhancement Skills (P2)**: book-architecture, ai-augmented-teaching

**Architecture Note**: Skills use Claude Code's model-invoked pattern with progressive disclosure. Scripts are **explicitly invoked via Bash tool** in SKILL.md instructions (not auto-executed). Skill activation is **semantic** (Claude understands context), not keyword-based.

---

## Quick Overview

| Phase | Skill | Tasks | Priority | Purpose |
|-------|-------|-------|----------|---------|
| 1-2 | Setup & Foundation | 12 | Critical | Infrastructure for all skills |
| 3 | learning-objectives | 10 | P1 | Generate measurable learning outcomes |
| 4 | concept-scaffolding | 10 | P1 | Break down complex concepts progressively |
| 5 | code-example-generator | 10 | P1 | Create validated, runnable teaching examples |
| 6 | exercise-designer | 12 | P1 | Design evidence-based practice activities |
| 7 | technical-clarity | 11 | P1 | Review explanations for accessibility |
| 8 | assessment-builder | 12 | P1 | Build deep understanding assessments |
| 9 | book-architecture | 13 | P2 | Design comprehensive book structures |
| 10 | ai-augmented-teaching | 13 | P2 | Integrate AI collaboration into curriculum |
| 11-12 | Integration & Polish | 21 | Final | Cross-skill validation and refinement |

**Total: 124 tasks** | **MVP: 38 tasks** (Setup + Foundation + learning-objectives) | **P1 Complete: 77 tasks**

---

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (skill) this task belongs to (US1-US8)
- Include exact file paths in descriptions

## Path Conventions

Skills live in `.claude/skills/<skill-name>/` per Claude Code conventions:
- `SKILL.md` - Core instructions with frontmatter (name, description, allowed-tools)
- `reference/` - Documentation (markdown) - Layer 3, loaded when SKILL.md references them
- `templates/` - Output structures (YAML/markdown) - Layer 3, loaded when needed
- `scripts/` - Validation utilities (Python 3.13+) - Layer 3, invoked via Bash tool

Tests live in `tests/` at repository root.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and directory structure for all 6 skills

- [ ] T001 Create base `.claude/skills/` directory structure
- [ ] T002 [P] Create `tests/unit/` directory for script unit tests
- [ ] T003 [P] Create `tests/integration/` directory for skill activation tests
- [ ] T004 [P] Create `tests/fixtures/` directory for test data
- [ ] T005 [P] Configure Python 3.13+ environment with type checking (mypy/pyright)
- [ ] T006 [P] Configure code formatting (black, isort) and linting (ruff/pylint)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Shared standalone scripts that skills invoke via Bash tool

**⚠️ CRITICAL**: No skill implementation can begin until this phase is complete

**Architecture**: Shared scripts are **standalone executables** (not importable modules). Each skill invokes them via Bash tool: `python .claude/skills/_shared/script-name.py <args>`

- [ ] T007 Create `.claude/skills/_shared/` directory for common standalone scripts
- [ ] T008 [P] Implement `_shared/sandbox-executor.py` as standalone script for isolated Python code execution with security constraints (timeout=5s, temp directory, no network). Script accepts code file path as argument, outputs JSON with execution results (stdout, stderr, exit_code, timeout). Invocation: `python _shared/sandbox-executor.py code.py --timeout 5`
- [ ] T009 [P] Write unit tests for sandbox-executor.py in `tests/unit/test_sandbox.py`
- [ ] T010 [P] Implement `_shared/yaml-validator.py` as standalone script that validates YAML file against schema. Accepts schema path and data path as arguments, outputs validation results. Invocation: `python _shared/yaml-validator.py schema.yml data.yml`
- [ ] T011 [P] Implement `_shared/readability-analyzer.py` as standalone script that calculates Flesch-Kincaid grade level, sentence length, complexity for text. Accepts file path, outputs JSON with metrics. Invocation: `python _shared/readability-analyzer.py text.md`
- [ ] T012 [P] Write unit tests for shared scripts in `tests/unit/test_shared_scripts.py`

**Checkpoint**: Foundation ready - skill implementation can now begin in parallel

---

## Phase 3: User Story 1 - learning-objectives Skill (Priority: P1) 🎯 MVP

**Goal**: Enable educators to generate measurable learning outcomes aligned with Bloom's taxonomy

**Independent Test**: Educator inputs "Define objectives for Python decorators" → skill activates → produces 3-5 objectives with Bloom's levels, prerequisites, assessment methods

### Implementation for learning-objectives

- [ ] T013 Create `.claude/skills/learning-objectives/` directory structure (reference/, templates/, scripts/)
- [ ] T014 [US1] Create `learning-objectives/SKILL.md` with frontmatter:
  - `name: learning-objectives`
  - `description:` Semantic description explaining this skill generates measurable learning outcomes aligned with Bloom's taxonomy for educational content. Describe when to use (curriculum planning, lesson design, defining what students will achieve). DO NOT use keyword triggers - use contextual semantic description.
  - `allowed-tools: Read, Bash, Write`
- [ ] T015 [P] [US1] Create `learning-objectives/reference/blooms-taxonomy-programming.md` with Bloom's 6 levels (Remember, Understand, Apply, Analyze, Evaluate, Create) adapted to programming contexts with action verb lists for each level
- [ ] T016 [P] [US1] Create `learning-objectives/reference/prerequisite-analysis.md` with strategies for identifying prerequisite knowledge chains
- [ ] T017 [P] [US1] Create `learning-objectives/reference/assessment-methods.md` with formative/summative/performance assessment types mapped to Bloom's levels
- [ ] T018 [P] [US1] Create `learning-objectives/templates/learning-objective-template.yml` matching data-model.md schema
- [ ] T019 [US1] Implement `learning-objectives/scripts/validate-objectives.py` as standalone script that validates objective statements use action verbs from Bloom's taxonomy and are measurable (not vague language like "understand", "know about"). Accepts YAML file path, outputs validation report. Invocation: `python scripts/validate-objectives.py objectives.yml`
- [ ] T020 [US1] Write unit test for validate-objectives.py in `tests/unit/test_validate_objectives.py`
- [ ] T021 [US1] Update `learning-objectives/SKILL.md` with complete instructions:
  - **Purpose**: What this skill does
  - **When to Activate**: Contextual situations (NOT keyword triggers)
  - **Process**: Step-by-step including:
    - When to read reference docs (e.g., "Read reference/blooms-taxonomy-programming.md for guidance")
    - How to generate objectives
    - How to invoke validation script: `python .claude/skills/learning-objectives/scripts/validate-objectives.py <file>`
    - How to interpret script output and refine
  - **Output Format**: Expected structure
  - **Examples**: 2-3 example interactions
- [ ] T022 [US1] Write integration test in `tests/integration/test_learning_objectives_activation.py` with semantically varied prompts (e.g., "define learning goals", "what should students achieve", "create measurable outcomes") to validate Claude understands semantic context

**Checkpoint**: learning-objectives skill should be fully functional - test by requesting objective generation for a Python topic

---

## Phase 4: User Story 2 - concept-scaffolding Skill (Priority: P1)

**Goal**: Enable educators to break down complex concepts into progressive learning steps with cognitive load management

**Independent Test**: Educator inputs "Scaffold async/await" → skill activates → produces 3-7 steps with cognitive load warnings, prerequisite chains

### Implementation for concept-scaffolding

- [ ] T023 Create `.claude/skills/concept-scaffolding/` directory structure
- [ ] T024 [US2] Create `concept-scaffolding/SKILL.md` with frontmatter:
  - `name: concept-scaffolding`
  - `description:` Semantic description for breaking down complex programming concepts into progressive learning steps with cognitive load management. Use when educators need to teach advanced topics incrementally.
  - `allowed-tools: Read, Bash, Write`
- [ ] T025 [P] [US2] Create `concept-scaffolding/reference/cognitive-load-theory.md` with CLT principles (intrinsic/extraneous/germane load, working memory limits 7±2 chunks, strategies for load reduction)
- [ ] T026 [P] [US2] Create `concept-scaffolding/reference/scaffolding-strategies.md` with worked examples, faded guidance, chunking patterns, interim checkpoints
- [ ] T027 [P] [US2] Create `concept-scaffolding/reference/worked-examples.md` with effective worked example structures for programming education
- [ ] T028 [P] [US2] Create `concept-scaffolding/templates/scaffolding-plan-template.yml` matching data-model.md schema (max 7 steps, max 3 new concepts per step)
- [ ] T029 [US2] Implement `concept-scaffolding/scripts/assess-cognitive-load.py` as standalone script that calculates cognitive load score based on number of new concepts, prerequisites, and step complexity. Accepts scaffolding plan YAML, outputs load assessment with warnings. Invocation: `python scripts/assess-cognitive-load.py plan.yml`
- [ ] T030 [US2] Write unit test for assess-cognitive-load.py in `tests/unit/test_cognitive_load.py`
- [ ] T031 [US2] Update `concept-scaffolding/SKILL.md` with complete instructions including explicit references to load reference docs and script invocation pattern
- [ ] T032 [US2] Write integration test in `tests/integration/test_concept_scaffolding_activation.py` with varied semantic requests

**Checkpoint**: concept-scaffolding skill should produce progressive breakdowns with cognitive load analysis

---

## Phase 5: User Story 3 - code-example-generator Skill (Priority: P1)

**Goal**: Enable authors to generate runnable, pedagogically sound code examples validated via sandbox execution

**Independent Test**: Author requests "Generate beginner example for list comprehensions" → skill activates → produces runnable code + explanation (what/how/why) + validation results

### Implementation for code-example-generator

- [ ] T033 Create `.claude/skills/code-example-generator/` directory structure
- [ ] T034 [US3] Create `code-example-generator/SKILL.md` with frontmatter:
  - `name: code-example-generator`
  - `description:` Semantic description for generating runnable, pedagogically sound Python code examples with validation. Use when authors need teaching examples that demonstrate concepts clearly.
  - `allowed-tools: Read, Bash, Write`
- [ ] T035 [P] [US3] Create `code-example-generator/reference/python-best-practices.md` with PEP 8 summary, type hints guidelines, docstring conventions, naming standards
- [ ] T036 [P] [US3] Create `code-example-generator/reference/example-patterns.md` with effective teaching example patterns (simple→complex progression, one concept per example, incremental complexity)
- [ ] T037 [P] [US3] Create `code-example-generator/reference/pep8-summary.md` with key PEP 8 style rules for generated code
- [ ] T038 [P] [US3] Create `code-example-generator/templates/code-example-template.md` with structure: fenced code block + explanation (what/how/why) + pedagogical notes
- [ ] T039 [US3] Implement `code-example-generator/scripts/validate-syntax.py` as standalone script using AST parsing to check Python syntax correctness. Accepts code file path, outputs syntax validation results. Invocation: `python scripts/validate-syntax.py code.py`
- [ ] T040 [US3] Write unit tests for validate-syntax.py in `tests/unit/test_syntax_validator.py`
- [ ] T041 [US3] Update `code-example-generator/SKILL.md` with complete instructions including:
  - How to generate examples following best practices
  - When to invoke syntax validation: `python scripts/validate-syntax.py code.py`
  - When to invoke sandbox execution: `python .claude/skills/_shared/sandbox-executor.py code.py --timeout 5`
  - How to interpret execution results and refine if errors occur
- [ ] T042 [US3] Write integration test in `tests/integration/test_code_example_generator_activation.py` validating end-to-end example generation + validation

**Checkpoint**: code-example-generator should produce validated, runnable examples with >95% success rate (SC-007)

---

## Phase 6: User Story 4 - exercise-designer Skill (Priority: P1)

**Goal**: Enable educators to design deliberate practice exercises applying evidence-based learning strategies

**Independent Test**: Educator provides learning objectives → skill activates → produces 3-5 varied exercises with difficulty progression, identified learning strategies, test cases

### Implementation for exercise-designer

- [ ] T043 Create `.claude/skills/exercise-designer/` directory structure
- [ ] T044 [US4] Create `exercise-designer/SKILL.md` with frontmatter:
  - `name: exercise-designer`
  - `description:` Semantic description for designing practice exercises applying evidence-based strategies (retrieval practice, spaced repetition, interleaving). Use when educators need varied exercise types targeting learning objectives.
  - `allowed-tools: Read, Bash, Write`
- [ ] T045 [P] [US4] Create `exercise-designer/reference/exercise-types.md` with fill-in-blank, debug-this, build-from-scratch, extend-code patterns with pedagogical guidance
- [ ] T046 [P] [US4] Create `exercise-designer/reference/evidence-based-strategies.md` with retrieval practice, spaced repetition, interleaving, desirable difficulty principles
- [ ] T047 [P] [US4] Create `exercise-designer/reference/difficulty-progression.md` with guidelines for gradual difficulty increase across exercise sets
- [ ] T048 [P] [US4] Create `exercise-designer/reference/spaced-repetition.md` with spacing intervals and repetition patterns for skill retention
- [ ] T049 [P] [US4] Create `exercise-designer/templates/exercise-template.yml` matching data-model.md schema (includes test cases, rubric)
- [ ] T050 [P] [US4] Create `exercise-designer/templates/rubric-template.yml` with criterion/points/descriptors structure
- [ ] T051 [US4] Implement `exercise-designer/scripts/generate-test-cases.py` as standalone script to create input/output test case pairs for exercises. Accepts exercise description, outputs test cases. Invocation: `python scripts/generate-test-cases.py exercise.yml`
- [ ] T052 [US4] Write unit test for generate-test-cases.py in `tests/unit/test_generate_test_cases.py`
- [ ] T053 [US4] Update `exercise-designer/SKILL.md` with complete instructions including when to reference strategies docs and how to invoke test case generation script
- [ ] T054 [US4] Write integration test in `tests/integration/test_exercise_designer_activation.py`

**Checkpoint**: exercise-designer should produce varied exercises with evidence-based strategies identified

---

## Phase 7: User Story 5 - technical-clarity Skill (Priority: P1)

**Goal**: Enable authors to review technical explanations for jargon, clarity, completeness, and readability

**Independent Test**: Author submits draft explanation → skill activates → produces clarity report with jargon identified, readability metrics, suggestions for improvement

### Implementation for technical-clarity

- [ ] T055 Create `.claude/skills/technical-clarity/` directory structure
- [ ] T056 [US5] Create `technical-clarity/SKILL.md` with frontmatter:
  - `name: technical-clarity`
  - `description:` Semantic description for reviewing technical explanations for jargon, readability, completeness, and accessibility. Use when authors need feedback on clarity of technical writing for learners.
  - `allowed-tools: Read, Bash, Write`
- [ ] T057 [P] [US5] Create `technical-clarity/reference/readability-metrics.md` with Flesch-Kincaid grade level, sentence length guidelines, vocabulary complexity measures
- [ ] T058 [P] [US5] Create `technical-clarity/reference/analogy-patterns.md` with effective analogy structures for technical concept explanation
- [ ] T059 [P] [US5] Create `technical-clarity/reference/clarity-checklist.md` with systematic clarity review criteria (jargon identification, context completeness, assumption detection)
- [ ] T060 [P] [US5] Create `technical-clarity/reference/gatekeeping-language.md` with examples of ableist/assumptive terms to avoid ("obviously", "simply", "just", "easy", "trivial")
- [ ] T061 [P] [US5] Create `technical-clarity/templates/clarity-report-template.yml` matching data-model.md schema
- [ ] T062 [US5] Implement `technical-clarity/scripts/check-completeness.py` as standalone script to identify missing critical information in technical explanations. Accepts text file, outputs completeness analysis. Invocation: `python scripts/check-completeness.py explanation.md`
- [ ] T063 [US5] Write unit tests for check-completeness.py in `tests/unit/test_completeness_checker.py`
- [ ] T064 [US5] Update `technical-clarity/SKILL.md` with complete instructions including:
  - When to reference readability metrics and gatekeeping language docs
  - How to invoke readability analysis: `python .claude/skills/_shared/readability-analyzer.py text.md`
  - How to invoke completeness check: `python scripts/check-completeness.py text.md`
  - How to compile clarity report with findings and suggestions
- [ ] T065 [US5] Write integration test in `tests/integration/test_technical_clarity_activation.py`

**Checkpoint**: technical-clarity should identify >80% of clarity issues when validated against expert review (SC-009)

---

## Phase 8: User Story 6 - assessment-builder Skill (Priority: P1)

**Goal**: Enable educators to create assessments measuring deep understanding with meaningful distractors and rubrics

**Independent Test**: Educator provides learning objectives → skill activates → produces varied questions (MCQ, code-completion, debugging, projects) with distractors based on misconceptions

### Implementation for assessment-builder

- [ ] T066 Create `.claude/skills/assessment-builder/` directory structure
- [ ] T067 [US6] Create `assessment-builder/SKILL.md` with frontmatter:
  - `name: assessment-builder`
  - `description:` Semantic description for creating assessments with varied question types (MCQ, code-completion, debugging) aligned to learning objectives with meaningful distractors. Use when educators design quizzes or exams measuring understanding.
  - `allowed-tools: Read, Bash, Write`
- [ ] T068 [P] [US6] Create `assessment-builder/reference/question-types.md` with MCQ, code-completion, debugging, short-answer, project-based patterns with pedagogical guidance
- [ ] T069 [P] [US6] Create `assessment-builder/reference/distractor-design.md` with misconception-based distractor creation strategies (identify common errors, plausible wrong answers)
- [ ] T070 [P] [US6] Create `assessment-builder/reference/rubric-guidelines.md` with criterion design for open-ended questions and projects
- [ ] T071 [P] [US6] Create `assessment-builder/reference/blooms-assessment-alignment.md` mapping Bloom's cognitive levels to appropriate question types
- [ ] T072 [P] [US6] Create `assessment-builder/templates/assessment-template.yml` matching data-model.md schema
- [ ] T073 [P] [US6] Create `assessment-builder/templates/rubric-template.yml` with excellent/adequate/insufficient descriptors
- [ ] T074 [US6] Implement `assessment-builder/scripts/validate-assessment.py` as standalone script to check objective coverage, cognitive distribution (60%+ non-recall), distractor quality. Accepts assessment YAML, outputs validation report. Invocation: `python scripts/validate-assessment.py assessment.yml`
- [ ] T075 [US6] Write unit test for validate-assessment.py in `tests/unit/test_validate_assessment.py`
- [ ] T076 [US6] Update `assessment-builder/SKILL.md` with complete instructions including when to reference distractor design docs and how to invoke assessment validation
- [ ] T077 [US6] Write integration test in `tests/integration/test_assessment_builder_activation.py`

**Checkpoint**: assessment-builder should produce assessments with meaningful distractors and balanced cognitive distribution (60%+ non-recall per SC-010)

---

## Phase 9: User Story 7 - book-architecture Skill (Priority: P2)

**Goal**: Enable authors to design cohesive book structures balancing tutorial progression with reference accessibility

**Independent Test**: Author planning comprehensive Python book → skill activates → produces chapter outline with flow patterns, dependency analysis, structural recommendations

### Implementation for book-architecture

- [ ] T078 Create `.claude/skills/book-architecture/` directory structure
- [ ] T079 [US7] Create `book-architecture/SKILL.md` with frontmatter:
  - `name: book-architecture`
  - `description:` Semantic description for designing comprehensive book structures with chapter flow, content organization, and balance between tutorial progression and reference material. Use when authors plan multi-chapter books or course curricula requiring cohesive structural design.
  - `allowed-tools: Read, Bash, Write`
- [ ] T080 [P] [US7] Create `book-architecture/reference/chapter-flow-patterns.md` with linear vs spiral curriculum patterns, chapter dependency strategies, content sequencing principles
- [ ] T081 [P] [US7] Create `book-architecture/reference/structural-patterns.md` with tutorial vs reference balancing, part/section organization, appendix design
- [ ] T082 [P] [US7] Create `book-architecture/reference/content-organization.md` with chunking strategies for large content, cross-referencing patterns, index design
- [ ] T083 [P] [US7] Create `book-architecture/reference/chapter-dependencies.md` with prerequisite mapping across chapters, optional vs core chapter identification
- [ ] T084 [P] [US7] Create `book-architecture/templates/book-outline-template.yml` with part/chapter/section hierarchy, learning trajectory
- [ ] T085 [P] [US7] Create `book-architecture/templates/chapter-structure-template.yml` with intro/body/summary/exercises structure
- [ ] T086 [US7] Implement `book-architecture/scripts/validate-structure.py` as standalone script that checks chapter dependency cycles, validates prerequisite chains, identifies orphaned content. Accepts book outline YAML, outputs structural analysis. Invocation: `python scripts/validate-structure.py outline.yml`
- [ ] T087 [US7] Implement `book-architecture/scripts/analyze-flow.py` as standalone script that evaluates chapter flow for cognitive progression, identifies difficulty jumps, suggests reordering. Invocation: `python scripts/analyze-flow.py outline.yml`
- [ ] T088 [US7] Write unit tests for structure validation scripts in `tests/unit/test_book_structure.py`
- [ ] T089 [US7] Update `book-architecture/SKILL.md` with complete instructions including:
  - When to reference flow patterns and organizational strategies
  - How to invoke structure validation: `python scripts/validate-structure.py outline.yml`
  - How to invoke flow analysis: `python scripts/analyze-flow.py outline.yml`
  - How to interpret analysis results and refine structure
- [ ] T090 [US7] Write integration test in `tests/integration/test_book_architecture_activation.py` with varied book planning requests

**Checkpoint**: book-architecture skill should produce cohesive book structures with validated chapter dependencies and flow analysis

---

## Phase 10: User Story 8 - ai-augmented-teaching Skill (Priority: P2)

**Goal**: Enable educators to design learning experiences that prepare students for AI-assisted development workflows

**Independent Test**: Educator designing AI-enhanced Python curriculum → skill activates → produces lesson plans integrating prompt engineering, AI pair programming patterns, AI tool literacy

### Implementation for ai-augmented-teaching

- [ ] T091 Create `.claude/skills/ai-augmented-teaching/` directory structure
- [ ] T092 [US8] Create `ai-augmented-teaching/SKILL.md` with frontmatter:
  - `name: ai-augmented-teaching`
  - `description:` Semantic description for designing learning experiences that prepare students for AI-assisted development, including prompt engineering pedagogy, AI pair programming, and AI tool literacy. Use when educators integrate AI collaboration tools into programming curriculum.
  - `allowed-tools: Read, Bash, Write`
- [ ] T093 [P] [US8] Create `ai-augmented-teaching/reference/prompt-engineering-pedagogy.md` with teaching effective prompting, prompt iteration strategies, specificity vs creativity tradeoffs
- [ ] T094 [P] [US8] Create `ai-augmented-teaching/reference/ai-pair-programming-patterns.md` with AI as explainer, AI as debugger, AI as code reviewer, AI as pair programmer roles
- [ ] T095 [P] [US8] Create `ai-augmented-teaching/reference/ai-tool-literacy.md` with understanding AI capabilities/limitations, verification strategies, over-reliance prevention, critical evaluation skills
- [ ] T096 [P] [US8] Create `ai-augmented-teaching/reference/ethical-ai-use.md` with attribution, academic integrity, bias awareness, responsible AI assistance guidelines
- [ ] T097 [P] [US8] Create `ai-augmented-teaching/templates/ai-lesson-template.yml` with AI integration points, learning objectives for AI skills, verification checkpoints
- [ ] T098 [P] [US8] Create `ai-augmented-teaching/templates/prompt-design-template.md` with effective prompt structure, common patterns, anti-patterns
- [ ] T099 [US8] Implement `ai-augmented-teaching/scripts/assess-ai-integration.py` as standalone script that evaluates appropriateness of AI integration in lesson, checks balance between AI assistance and foundational learning. Accepts lesson plan YAML, outputs integration assessment. Invocation: `python scripts/assess-ai-integration.py lesson.yml`
- [ ] T100 [US8] Implement `ai-augmented-teaching/scripts/validate-prompts.py` as standalone script that analyzes prompt quality (specificity, clarity, context), suggests improvements. Invocation: `python scripts/validate-prompts.py prompts.yml`
- [ ] T101 [US8] Write unit tests for AI integration assessment scripts in `tests/unit/test_ai_assessment.py`
- [ ] T102 [US8] Update `ai-augmented-teaching/SKILL.md` with complete instructions including:
  - When to reference prompt pedagogy and AI literacy docs
  - How to invoke AI integration assessment: `python scripts/assess-ai-integration.py lesson.yml`
  - How to invoke prompt validation: `python scripts/validate-prompts.py prompts.yml`
  - How to balance AI assistance with foundational skill development
- [ ] T103 [US8] Write integration test in `tests/integration/test_ai_augmented_teaching_activation.py` with varied AI curriculum design requests

**Checkpoint**: ai-augmented-teaching skill should produce AI-integrated lesson plans with balanced assistance and foundational skill development

---

## Phase 11: Integration & Sequential Activation Testing

**Purpose**: Validate multi-skill workflows and semantic activation patterns across all 8 skills

- [ ] T104 Write integration test in `tests/integration/test_sequential_skills.py` for "generate example and review for clarity" (code-example-generator → technical-clarity)
- [ ] T105 [P] Write integration test for "create objectives and design exercises" (learning-objectives → exercise-designer)
- [ ] T106 [P] Write integration test for "scaffold concept and generate examples" (concept-scaffolding → code-example-generator)
- [ ] T107 [P] Write integration test for "design book structure and define chapter objectives" (book-architecture → learning-objectives)
- [ ] T108 [P] Write integration test for "AI-augmented lesson with code examples" (ai-augmented-teaching → code-example-generator)
- [ ] T109 Verify mutually exclusive semantic descriptions prevent activation conflicts by testing edge cases where multiple skills could be relevant (test all 8 skills)
- [ ] T110 Test skill activation with semantically varied prompts (target >80% activation success per SC-003). Test all 8 skills with 3-5 varied phrasings each

---

## Phase 12: Polish & Cross-Cutting Concerns

**Purpose**: Improvements affecting all 8 skills and final validation

- [ ] T111 [P] Validate all SKILL.md frontmatter against `contracts/skill-metadata.schema.yml` (all 8 skills)
- [ ] T112 [P] Validate all YAML templates against their respective schemas in `contracts/`
- [ ] T113 [P] Run code formatter (black, isort) on all Python scripts
- [ ] T114 [P] Run type checker (mypy/pyright) on all Python scripts - must pass with no errors
- [ ] T115 [P] Run linter (ruff/pylint) on all Python scripts
- [ ] T116 Create `README.md` in `.claude/skills/` directory explaining skill organization, progressive disclosure architecture, and all 8 skills' purposes
- [ ] T117 Update `quickstart.md` with implementation-specific details for all 8 skills: how scripts are invoked via Bash, how reference docs are loaded, troubleshooting common issues
- [ ] T118 Add troubleshooting section to quickstart.md based on integration test findings (e.g., "Skill not activating? Check description is contextual, not keyword-based")
- [ ] T119 Verify all file paths in quickstart.md match actual implementation
- [ ] T120 Run all unit tests (`pytest tests/unit/`) - must pass 100%
- [ ] T121 Run all integration tests (`pytest tests/integration/`) - must pass with >80% skill activation accuracy
- [ ] T122 Performance validation: Measure response times for each of 8 skills against targets
- [ ] T123 Create skill activation reference document explaining semantic activation patterns vs keywords for all 8 skills
- [ ] T124 Final validation: Test each of 8 skills independently per user story acceptance scenarios

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all skills
- **Core Skills US1-US6 (Phases 3-8)**: All depend on Foundational phase completion (Priority P1)
  - Skills can then proceed in parallel (6 developers, 6 skills simultaneously)
  - Or sequentially in priority order
- **Enhancement Skills US7-US8 (Phases 9-10)**: Depend on Foundational phase completion (Priority P2)
  - Can run in parallel with P1 skills if resources available
  - Or implement after P1 skills complete
- **Integration (Phase 11)**: Depends on at least 2 skills complete (recommend all 8 for complete validation)
- **Polish (Phase 12)**: Depends on all skills complete

### Skill (User Story) Dependencies

**P1 Core Skills (Independent after Foundational):**
- **US1 (learning-objectives)**: Can start after Foundational ✅ MVP candidate
- **US2 (concept-scaffolding)**: Can start after Foundational - Independent
- **US3 (code-example-generator)**: Can start after Foundational - Independent (uses sandbox-executor from Foundational)
- **US4 (exercise-designer)**: Can start after Foundational - Independent
- **US5 (technical-clarity)**: Can start after Foundational - Independent (uses readability-analyzer from Foundational)
- **US6 (assessment-builder)**: Can start after Foundational - Independent

**P2 Enhancement Skills (Independent after Foundational):**
- **US7 (book-architecture)**: Can start after Foundational - Independent
- **US8 (ai-augmented-teaching)**: Can start after Foundational - Independent

**KEY INSIGHT**: All 8 skills are completely independent after Foundational phase completes!

### Within Each Skill

1. Create directory structure
2. Create SKILL.md with frontmatter (name, semantic description, allowed-tools)
3. Create reference files (can be parallel)
4. Create templates (can be parallel)
5. Implement standalone scripts
6. Write unit tests for scripts
7. Update SKILL.md with complete instructions (including explicit script invocations and reference doc loading)
8. Write integration test for semantic skill activation

### Parallel Opportunities

**Phase 1 (Setup)**: T002, T003, T004, T005, T006 can all run in parallel

**Phase 2 (Foundational)**: T008, T009, T010, T011, T012 can run in parallel after T007

**Phase 3-10 (All 8 Skills)**:
- All 8 skill phases can run in parallel with 8 developers
- Can also run as P1 (6 skills) first, then P2 (2 skills) second
- Within each skill: reference files, templates, and unit tests can run in parallel
- Example for learning-objectives: T015, T016, T017, T018 can all run in parallel
- Example for book-architecture: T080, T081, T082, T083, T084, T085 can all run in parallel

**Phase 12 (Polish)**: T111, T112, T113, T114, T115 can all run in parallel

---

## Implementation Strategy

### MVP First (User Story 1 Only - learning-objectives)

1. Complete Phase 1: Setup (T001-T006)
2. Complete Phase 2: Foundational (T007-T012) **← CRITICAL BLOCKER**
3. Complete Phase 3: learning-objectives skill (T013-T022)
4. **STOP and VALIDATE**: Test learning-objectives independently
   - Try: "Define objectives for Python decorators" (semantic request)
   - Verify: 3-5 objectives produced, Bloom's levels applied, prerequisites identified
5. Deploy/demo if ready - **This is a viable MVP!**

### Incremental Delivery (Add Skills One at a Time)

**P1 Core Skills (Essential):**
1. Setup + Foundational → Foundation ready
2. Add learning-objectives → Test independently → Deploy/Demo (MVP!)
3. Add concept-scaffolding → Test independently → Deploy/Demo
4. Add code-example-generator → Test independently → Deploy/Demo
5. Add exercise-designer → Test independently → Deploy/Demo
6. Add technical-clarity → Test independently → Deploy/Demo
7. Add assessment-builder → Test independently → Deploy/Demo

**P2 Enhancement Skills (Optional):**
8. Add book-architecture → Test independently → Deploy/Demo (for comprehensive book projects)
9. Add ai-augmented-teaching → Test independently → Deploy/Demo (for AI-enhanced curriculum)

Each skill adds value without breaking previous skills.

### Parallel Team Strategy (8 Developers - Full Implementation)

With 8 developers for all 8 skills:

1. Team completes Setup (Phase 1) together (T001-T006)
2. Team completes Foundational (Phase 2) together (T007-T012) **← CRITICAL**
3. Once Foundational is done, **split into 8 parallel tracks**:
   - Developer A: learning-objectives (T013-T022)
   - Developer B: concept-scaffolding (T023-T032)
   - Developer C: code-example-generator (T033-T042)
   - Developer D: exercise-designer (T043-T054)
   - Developer E: technical-clarity (T055-T065)
   - Developer F: assessment-builder (T066-T077)
   - Developer G: book-architecture (T078-T090)
   - Developer H: ai-augmented-teaching (T091-T103)
4. All skills complete independently and simultaneously
5. Reconvene for Integration testing (Phase 11) and Polish (Phase 12)

**Timeline with 8 developers**: ~3-4 days
- Day 1: Setup + Foundational (everyone)
- Days 2-3: Parallel skill implementation (8 tracks)
- Day 4: Integration + Polish

**Timeline with 6 developers** (P1 first, then P2): ~4-5 days
- Day 1: Setup + Foundational (everyone)
- Days 2-3: P1 skills in parallel (6 tracks)
- Day 4: P2 skills in parallel (2 tracks, 4 developers idle or helping)
- Day 5: Integration + Polish

**Timeline with 1 developer**: ~14-16 days (sequential implementation of all 8 skills)

---

## Task Count Summary

- **Phase 1 (Setup)**: 6 tasks
- **Phase 2 (Foundational)**: 6 tasks (CRITICAL - blocks all skills)
- **Phase 3 (US1 - learning-objectives)**: 10 tasks [P1]
- **Phase 4 (US2 - concept-scaffolding)**: 10 tasks [P1]
- **Phase 5 (US3 - code-example-generator)**: 10 tasks [P1]
- **Phase 6 (US4 - exercise-designer)**: 12 tasks [P1]
- **Phase 7 (US5 - technical-clarity)**: 11 tasks [P1]
- **Phase 8 (US6 - assessment-builder)**: 12 tasks [P1]
- **Phase 9 (US7 - book-architecture)**: 13 tasks [P2]
- **Phase 10 (US8 - ai-augmented-teaching)**: 13 tasks [P2]
- **Phase 11 (Integration)**: 7 tasks
- **Phase 12 (Polish)**: 14 tasks

**Total**: 124 tasks (96 for P1 core + 28 for P2 enhancement)

**Parallel opportunities**: 50+ tasks marked [P] can run in parallel when conditions are met

**Breakdown by Priority**:
- **P1 Essential** (6 core skills): 77 tasks (Setup + Foundational + 6 skills)
- **P2 Enhancement** (2 additional skills): 26 tasks
- **Shared** (Integration + Polish): 21 tasks

---

## Success Criteria Validation

Each skill phase maps to success criteria from spec.md:

| Skill | Success Criteria | Validation Task |
|-------|------------------|-----------------|
| learning-objectives | SC-001, SC-005, SC-011 | T022 (integration test) |
| concept-scaffolding | SC-001, SC-006 | T032 (integration test) |
| code-example-generator | SC-001, SC-007, SC-012 | T042 (integration test) |
| exercise-designer | SC-001, SC-008, SC-013 | T054 (integration test) |
| technical-clarity | SC-001, SC-009, SC-014 | T065 (integration test) |
| assessment-builder | SC-001, SC-010 | T077 (integration test) |
| book-architecture | SC-001, SC-002 [P2] | T090 (integration test) |
| ai-augmented-teaching | SC-001, SC-002 [P2] | T103 (integration test) |
| All 8 skills | SC-002, SC-003, SC-004 | T109, T110 (activation tests) |
| All 8 skills | SC-015 | T124 (final validation) |

Performance validation (SC-005 to SC-010 for P1 skills) in T122.

---

## Key Architectural Corrections from Original

**✅ CORRECTED**:
1. **Skill activation**: Semantic context-based (not keyword triggers)
2. **Script execution**: Explicitly invoked via Bash tool in SKILL.md instructions (not auto-executed)
3. **Shared utilities**: Standalone invocable scripts (not importable modules)
4. **Descriptions**: Contextual semantic explanations (not trigger phrase lists)
5. **Frontmatter**: Added `allowed-tools` field to all skills
6. **Progressive disclosure**: Clarified Layer 1/2/3 architecture
7. **Integration tests**: Test semantic activation, not keyword matching

---

## Notes

- [P] tasks = different files, no dependencies on incomplete work
- [Story] label (US1-US8) maps task to specific skill for traceability
- Each of 8 skills is independently completable and testable
- Tests validate semantic skill activation and output correctness
- Foundational phase (T007-T012) is CRITICAL - nothing else can start until complete
- All 8 skills can be built in parallel after Foundational phase (or P1 first, then P2)
- Commit after each task or logical group
- Stop at any checkpoint to validate skill independently
- Hard failure mode (per clarification #1) means validation errors must be clear and actionable
- Scripts output structured data (JSON/YAML) that Claude interprets via SKILL.md instructions

---

## Summary: 8 Skills for Reusable Intelligence

This task list implements **8 comprehensive pedagogical skills** for Python educational content development:

**P1 Core Skills (Essential for Educational Content)**:
1. **learning-objectives** - Measurable outcomes aligned with Bloom's taxonomy
2. **concept-scaffolding** - Progressive concept breakdowns with cognitive load management
3. **code-example-generator** - Runnable, validated teaching examples
4. **exercise-designer** - Evidence-based practice activities
5. **technical-clarity** - Readability and accessibility review
6. **assessment-builder** - Deep understanding assessments with meaningful distractors

**P2 Enhancement Skills (For Comprehensive Projects)**:
7. **book-architecture** - Cohesive multi-chapter book structure design
8. **ai-augmented-teaching** - AI-assisted development curriculum integration

**Total Implementation**: 124 tasks across 12 phases, fully architected for Claude Code's semantic activation and progressive disclosure patterns.
