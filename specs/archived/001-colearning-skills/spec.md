# Feature Specification: Build Colearning Skills for Python Book Agent

**Feature Branch**: `001-colearning-skills`
**Created**: 2025-10-28
**Updated**: 2025-10-28
**Status**: Draft
**Input**: User description: "Build skills for Python colearning book agent. Refined architecture with 6 core focused skills: learning-objectives.md, concept-scaffolding.md, code-example-generator.md, exercise-designer.md, technical-clarity.md, assessment-builder.md. Optional: book-architecture.md, ai-augmented-teaching.md."

## Clarifications

### Session 2025-10-28

- Q: When a skill fails to activate or encounters an error during execution, how should it handle the failure? → A: Hard failure - skill halts execution and requires user intervention
- Q: What file formats should supporting files (reference docs, templates, scripts) use within each skill directory? → A: Mixed: markdown for docs, Python scripts for validation/analysis, YAML for structured data
- Q: How should the code-example-generator skill validate that generated Python code is runnable and correct? → A: Execution in sandbox - actually run code in isolated environment to verify output
- Q: How should skills standardize their output format to enable potential skill-to-skill integration? → A: No specific format prescribed
- Q: If multiple skills could apply to the same context, how should Claude handle potential skill conflicts? → A: Skills have mutually exclusive descriptions to prevent conflicts; when multiple skills are relevant for different purposes, they activate sequentially with combined outputs

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.
  
  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Educators Define Measurable Learning Objectives (Priority: P1)

Educators need to define clear, measurable learning outcomes for each lesson or chapter, aligned with established learning taxonomies (Bloom's, programming-specific competencies). They should understand what prerequisite knowledge is required and how to verify students achieved the objectives.

**Why this priority**: Learning objectives are the foundation of effective instruction. Without clear objectives, content creation, exercise design, and assessment become unfocused and ineffective.

**Independent Test**: Can be fully tested by an educator defining learning objectives for a Python chapter (e.g., "Functions and Scope") and verifying that the learning-objectives skill produces measurable outcomes, identifies prerequisites, and suggests verification methods.

**Acceptance Scenarios**:

1. **Given** an educator inputs a topic (e.g., "list comprehensions"), **When** they invoke the learning-objectives skill, **Then** the system produces 3-5 measurable learning outcomes with appropriate Bloom's taxonomy levels and prerequisite concept analysis.
2. **Given** draft learning objectives exist, **When** an educator requests review, **Then** the system identifies vague objectives (e.g., "understand loops") and suggests measurable alternatives (e.g., "write for-loops to iterate over sequences and explain iteration flow").

---

### User Story 2 - Educators Scaffold Complex Programming Concepts (Priority: P1)

Educators need to break down complex Python concepts (OOP, decorators, async/await, metaclasses) into learnable chunks with appropriate cognitive load management. They should understand how to introduce concepts progressively with worked examples and interim checkpoints.

**Why this priority**: Complex concepts cause learner overwhelm and dropout. Effective scaffolding is critical for retention and mastery.

**Independent Test**: Can be fully tested by an educator planning how to teach a complex topic (e.g., "async programming") and the concept-scaffolding skill producing a progressive introduction strategy with cognitive load considerations.

**Acceptance Scenarios**:

1. **Given** a complex concept (e.g., "decorators with arguments"), **When** the concept-scaffolding skill is invoked, **Then** the system produces a 3-5 step progression starting with simple cases, building to the full concept, with cognitive load warnings.
2. **Given** a scaffolding plan exists, **When** reviewed for effectiveness, **Then** the system verifies each step builds on prior knowledge and no step introduces too many new ideas simultaneously.

---

### User Story 3 - Authors Generate Pedagogically Sound Code Examples (Priority: P1)

Authors need to create runnable code examples that clearly demonstrate specific concepts without confusion. Examples should be incremental, well-commented, follow Python best practices, and demonstrate one concept at a time.

**Why this priority**: Code examples are the primary teaching tool in programming education. Poor examples cause confusion and reinforce bad practices.

**Independent Test**: Can be fully tested by an author requesting example code for a concept (e.g., "context managers") and the code-example-generator skill producing runnable, well-structured teaching examples.

**Acceptance Scenarios**:

1. **Given** a learning objective (e.g., "demonstrate try/except error handling"), **When** the code-example-generator skill is invoked, **Then** the system produces 2-3 progressive examples that are runnable, commented, and demonstrate the concept clearly.
2. **Given** an existing code example, **When** reviewed for pedagogical soundness, **Then** the system identifies issues (too complex, multiple concepts, poor naming, missing comments) and suggests improvements.

---

### User Story 4 - Educators Design Deliberate Practice Exercises (Priority: P1)

Educators need to create coding exercises and projects that apply deliberate practice principles—spaced repetition, interleaving, retrieval practice, and appropriate difficulty progression. Exercises should target specific learning objectives and provide meaningful feedback opportunities.

**Why this priority**: Practice is where learning solidifies. Poor exercise design leads to rote memorization instead of skill development.

**Independent Test**: Can be fully tested by an educator designing practice exercises for a lesson and the exercise-designer skill evaluating whether exercises apply evidence-based learning strategies and target objectives.

**Acceptance Scenarios**:

1. **Given** learning objectives for a lesson, **When** the exercise-designer skill is invoked, **Then** the system produces 3-5 varied exercise types (fill-in-blank, debug-this, build-from-scratch) with difficulty progression and spaced repetition opportunities.
2. **Given** draft exercises exist, **When** reviewed for pedagogical effectiveness, **Then** the system identifies which exercises apply retrieval practice, interleaving, and appropriate challenge levels.

---

### User Story 5 - Authors Review Explanations for Technical Clarity (Priority: P1)

Authors need feedback on whether their technical explanations are clear, accessible, and complete for the target audience level. They should identify jargon, missing context, weak analogies, and opportunities to improve readability.

**Why this priority**: Clear communication is essential for learning. Dense, jargon-heavy explanations create barriers and frustration.

**Independent Test**: Can be fully tested by an author submitting a draft explanation and the technical-clarity skill providing specific clarity feedback with improvement suggestions.

**Acceptance Scenarios**:

1. **Given** a draft technical explanation (e.g., "how Python's GIL works"), **When** the technical-clarity skill is invoked, **Then** the system identifies jargon, unclear passages, missing context, and suggests analogies or simplifications.
2. **Given** an explanation targeting beginners, **When** evaluated for accessibility, **Then** the system verifies readability scores are appropriate and flags advanced terminology without definitions.

---

### User Story 6 - Educators Create Assessments for Deep Understanding (Priority: P1)

Educators need to design quizzes, tests, and assessments that measure conceptual understanding (not just memorization). Assessments should align with learning objectives, include meaningful distractors, and provide opportunities for feedback.

**Why this priority**: Assessments drive student learning behavior. Poor assessments (trivial recall questions) encourage surface learning instead of mastery.

**Independent Test**: Can be fully tested by an educator creating assessment questions for a chapter and the assessment-builder skill evaluating whether questions target understanding and align with objectives.

**Acceptance Scenarios**:

1. **Given** learning objectives for a chapter, **When** the assessment-builder skill is invoked, **Then** the system produces varied question types (MCQ with meaningful distractors, code-completion, debugging challenges, open-ended) that target conceptual understanding.
2. **Given** draft assessment questions exist, **When** reviewed for quality, **Then** the system identifies trivial recall questions, suggests deeper conceptual questions, and validates that distractors represent common misconceptions.

### Edge Cases

- What happens when an educator has a non-standard Python curriculum (e.g., data science-focused, web development, DevOps automation) rather than general-purpose Python?
- How do skills handle very short courses (1-3 lessons) versus comprehensive textbooks (30+ chapters)?
- What if technical content needs to explain deprecated Python 2 features or legacy code patterns for maintenance contexts?
- How should exercise-designer and assessment-builder skills adapt to different learning modalities (self-paced online, classroom, bootcamp, university course)?
- What happens when learning objectives conflict with time constraints (educator wants depth but has limited course hours)?
- How does code-example-generator handle platform-specific code (Windows vs. Linux vs. macOS) or version-specific Python features?
- What if an educator wants to teach "bad practices" explicitly (to show what NOT to do) - how do skills handle pedagogically intentional anti-patterns?
- How do skills handle interdisciplinary content (Python for scientists, Python for artists, Python for kids)?
- How does sandbox execution handle code examples requiring external packages not in standard library?
- What happens if sandbox execution times out or consumes excessive resources during code validation?

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

**Learning-Objectives Skill (learning-objectives.md)**
- **FR-001**: Learning-objectives skill MUST generate 3-5 measurable learning outcomes for a given topic, aligned with appropriate Bloom's taxonomy levels (remember, understand, apply, analyze, evaluate, create).
- **FR-002**: Learning-objectives skill MUST analyze and document prerequisite knowledge required before each learning objective can be achieved.
- **FR-003**: Learning-objectives skill MUST identify vague or unmeasurable objectives and suggest specific, verifiable alternatives.
- **FR-004**: Learning-objectives skill MUST map learning objectives to appropriate assessment methods (formative vs. summative, knowledge checks vs. performance tasks).

**Concept-Scaffolding Skill (concept-scaffolding.md)**
- **FR-005**: Concept-scaffolding skill MUST break down complex concepts into 3-7 progressive learning steps, each building on prior knowledge.
- **FR-006**: Concept-scaffolding skill MUST assess cognitive load at each step and warn when too many new ideas are introduced simultaneously.
- **FR-007**: Concept-scaffolding skill MUST suggest worked examples, interim checkpoints, and fading support strategies for each scaffolding step.
- **FR-008**: Concept-scaffolding skill MUST identify prerequisite concepts that must be mastered before introducing the target complex concept.

**Code-Example-Generator Skill (code-example-generator.md)**
- **FR-009**: Code-example-generator skill MUST produce runnable Python code examples that demonstrate one concept clearly per example.
- **FR-010**: Code-example-generator skill MUST include meaningful comments explaining what the code does and why, suitable for the target learning level.
- **FR-011**: Code-example-generator skill MUST validate generated code by executing it in an isolated sandbox environment to verify it runs without errors and produces expected output. Code MUST also follow Python best practices (PEP 8, meaningful names, appropriate structure).
- **FR-012**: Code-example-generator skill MUST create progressive example sequences (simple → intermediate → full complexity) for complex concepts.
- **FR-013**: Code-example-generator skill MUST flag examples that introduce multiple new concepts simultaneously and suggest simplification.

**Exercise-Designer Skill (exercise-designer.md)**
- **FR-014**: Exercise-designer skill MUST create varied exercise types (fill-in-blank, debug-this, build-from-scratch, extend-code) targeting specific learning objectives.
- **FR-015**: Exercise-designer skill MUST identify which evidence-based learning strategies each exercise employs (retrieval practice, spaced repetition, interleaving, desirable difficulty).
- **FR-016**: Exercise-designer skill MUST suggest difficulty progression across exercises, ensuring appropriate challenge levels.
- **FR-017**: Exercise-designer skill MUST provide guidance on creating test cases or rubrics for evaluating student solutions.
- **FR-018**: Exercise-designer skill MUST identify opportunities for spaced repetition (revisiting prior concepts) within exercise sets.

**Technical-Clarity Skill (technical-clarity.md)**
- **FR-019**: Technical-clarity skill MUST analyze technical explanations and identify jargon, unclear passages, and missing context.
- **FR-020**: Technical-clarity skill MUST suggest analogies, simplifications, or alternative phrasings to improve accessibility for the target audience level.
- **FR-021**: Technical-clarity skill MUST assess completeness, identifying critical missing information or unaddressed edge cases.
- **FR-022**: Technical-clarity skill MUST evaluate readability using objective metrics (sentence length, complexity, vocabulary level) and suggest improvements.
- **FR-023**: Technical-clarity skill MUST validate that code snippets within explanations are accurate and support the explanation.

**Assessment-Builder Skill (assessment-builder.md)**
- **FR-024**: Assessment-builder skill MUST generate varied question types (multiple-choice, code-completion, debugging, short-answer, project-based) aligned with learning objectives.
- **FR-025**: Assessment-builder skill MUST create multiple-choice questions with meaningful distractors based on common student misconceptions.
- **FR-026**: Assessment-builder skill MUST distinguish between questions testing recall vs. conceptual understanding vs. application/synthesis.
- **FR-027**: Assessment-builder skill MUST suggest rubrics or evaluation criteria for open-ended questions and projects.
- **FR-028**: Assessment-builder skill MUST identify trivial questions (pure memorization) and suggest alternatives that test deeper understanding.

**Integration & Delivery (All Skills)**
- **FR-029**: All six core skills MUST integrate with Claude Code as model-invoked skills with proper SKILL.md frontmatter (name, description, allowed-tools). Skill activation is **semantic** (Claude understands context and intent), NOT keyword-based (no trigger phrase matching). Skill descriptions MUST be mutually exclusive using contextual semantic differentiation to prevent activation conflicts - each description should clearly define the specific context and purpose that triggers that skill alone.
- **FR-030**: All skills MUST accept content input in markdown or plain text format. Skill output format is NOT prescribed - each skill MAY use the most appropriate format for its purpose (freeform markdown, structured sections, YAML frontmatter, etc.) to maximize usability and clarity.
- **FR-031**: All skills MUST provide actionable, specific feedback with examples (not just generic criticism).
- **FR-032**: Each skill MUST include supporting reference files using progressive disclosure (loaded only when SKILL.md references them). Supporting files MUST use appropriate formats: markdown (.md) for documentation and reference materials, Python (.py) for validation/analysis scripts (invoked via Bash tool, NOT auto-executed), YAML (.yml/.yaml) for structured data and configuration. SKILL.md instructions MUST explicitly direct Claude when to load reference docs and when to invoke scripts via Bash tool.
- **FR-033**: Skills MUST be independently usable (no required dependencies on other skills) but MAY reference outputs from other skills when available.
- **FR-035**: When multiple skills are relevant for different purposes (e.g., generating code examples AND reviewing them for clarity), Claude MUST activate skills sequentially and combine outputs coherently for the user.

**Error Handling**
- **FR-034**: Skills MUST implement hard failure mode - when a skill fails to activate or encounters an error during execution, it MUST halt execution and require user intervention with a clear error message describing the failure cause and suggested remediation steps.

### Key Entities *(include if feature involves data)*

- **Learning Objective**: A measurable outcome statement describing what a learner will be able to do, aligned with Bloom's taxonomy level, with identified prerequisites and assessment methods. Output of learning-objectives skill.

- **Scaffolding Plan**: A progressive breakdown of a complex concept into 3-7 learning steps, each with cognitive load assessment, worked examples, and support strategies. Output of concept-scaffolding skill.

- **Code Example**: A runnable Python code snippet with explanatory comments, demonstrating one concept clearly, validated for correctness and pedagogical soundness. Output of code-example-generator skill.

- **Exercise Set**: A collection of varied practice activities (fill-in-blank, debug-this, build-from-scratch) with difficulty progression, identified learning strategies, and evaluation guidance. Output of exercise-designer skill.

- **Clarity Report**: Analysis of a technical explanation identifying jargon, unclear passages, missing context, readability metrics, and improvement suggestions. Output of technical-clarity skill.

- **Assessment Package**: A collection of questions/tasks (MCQ, code-completion, projects) with distractors, rubrics, and alignment to learning objectives and cognitive levels. Output of assessment-builder skill.

- **Skill Invocation Context**: Metadata about skill activation including target audience level (beginner/intermediate/advanced), learning modality (self-paced/classroom), and curriculum focus (general Python/data science/web dev).

## Success Criteria *(mandatory)*

### Measurable Outcomes

**Skill Implementation & Integration**
- **SC-001**: Each of the six core skills (learning-objectives, concept-scaffolding, code-example-generator, exercise-designer, technical-clarity, assessment-builder) is implemented as a callable Claude Code skill with proper SKILL.md frontmatter.
- **SC-002**: Each skill includes at least 2-3 supporting files organized for progressive disclosure (reference docs, templates, scripts).
- **SC-003**: All six skills are model-invoked (Claude automatically activates based on context) with activation success rate >80% when appropriate context is present.
- **SC-004**: Skills are discoverable through Claude Code's skill system and documented with clear activation triggers and usage examples.

**Skill Performance & Quality**
- **SC-005**: Learning-objectives skill generates 3-5 measurable objectives with Bloom's taxonomy alignment and prerequisite analysis in <30 seconds for typical topics.
- **SC-006**: Concept-scaffolding skill produces 3-7 step breakdowns for complex concepts with cognitive load warnings within 45 seconds.
- **SC-007**: Code-example-generator skill produces runnable, correct Python examples with >95% success rate (validated by sandbox execution - code runs without errors and produces expected output).
- **SC-008**: Exercise-designer skill creates 3-5 varied exercise types with identified learning strategies in <60 seconds.
- **SC-009**: Technical-clarity skill identifies >80% of clarity issues (jargon, unclear passages, missing context) when validated against expert review.
- **SC-010**: Assessment-builder skill generates questions with meaningful distractors that align with learning objectives at appropriate cognitive levels.

**User Impact & Effectiveness**
- **SC-011**: Educators using learning-objectives skill report reduction in time spent defining learning outcomes (baseline vs. post-implementation survey).
- **SC-012**: Code examples generated by code-example-generator skill receive positive pedagogical ratings from expert reviewers (>4/5 on clarity, correctness, instructional value).
- **SC-013**: Exercises created with exercise-designer skill demonstrate application of at least 2 evidence-based learning strategies per exercise set.
- **SC-014**: Technical explanations reviewed by technical-clarity skill show measurable improvement in readability scores (Flesch-Kincaid grade level reduction for beginner content).
- **SC-015**: Educators using all six skills together report improved confidence in creating pedagogically sound learning materials (measured via survey).

## Assumptions

- **Learning level**: Content targets beginner to intermediate Python learners by default; skills adapt recommendations based on specified audience level (beginner/intermediate/advanced).
- **Technical accuracy**: Code-example-generator validates syntax and basic correctness; other skills assume technical content accuracy and focus on pedagogical soundness.
- **Integration model**: Skills integrate as model-invoked Claude Code skills using progressive disclosure; no external databases or APIs required.
- **Evidence-based pedagogy**: Skill recommendations are based on established educational research (Bloom's taxonomy, cognitive load theory, spacing effect, retrieval practice, interleaving, desirable difficulty).
- **Platform assumptions**: Code examples assume Python 3.8+ unless specified; platform-specific code (OS-dependent) is flagged for educator review.
- **Skill independence**: Each skill operates independently but MAY consume outputs from other skills when provided (e.g., assessment-builder can reference learning-objectives output).
- **Progressive disclosure**: Supporting files (reference docs, templates, scripts) are loaded only when skill is activated and context requires them.

## Dependencies & Constraints

- **Skill framework**: Implementation depends on Claude Code skill infrastructure (SKILL.md format, model invocation, progressive file loading).
- **Python domain knowledge**: Skills embed Python-specific pedagogy and best practices; supporting files contain reference materials (PEP 8, common patterns, learning progressions).
- **No external dependencies**: Skills operate on provided content without external APIs, databases, or services (stateless operation).
- **Context window constraints**: Supporting files use progressive disclosure to minimize context usage; only relevant reference materials are loaded per invocation.
- **Code validation sandbox**: Code-example-generator MUST use an isolated sandbox environment (e.g., Docker container, Python subprocess with restricted permissions, or similar isolation mechanism) to execute and validate generated code examples. Sandbox MUST prevent file system access outside designated temp directories, network access, and resource overconsumption (CPU/memory limits enforced).
- **Single invocation model**: Skills provide complete responses per invocation; do not maintain state across interactions (educators may provide prior skill outputs as input context).
- **Skill file organization**: Each skill stored in `.claude/skills/<skill-name>/` with SKILL.md + supporting files organized by type: reference/ (markdown docs), templates/ (markdown/YAML templates), scripts/ (Python validation/analysis tools).
- **Version compatibility**: Skills target Claude Code with model-invoked skill support (Claude 3.5 Sonnet or newer).

## Out of Scope

- Implementation of an automated learning management system (LMS) or student enrollment/tracking system.
- Automatic grading or evaluation of actual student code submissions (exercise-designer and assessment-builder provide rubrics and guidelines only).
- Real-time collaboration features or multi-user editing (skills are single-user, stateless operations).
- Integration with specific publishing platforms, textbook systems, or educational technology vendors.
- Translation or localization to non-English languages (skills operate in English).
- Full curriculum planning or accreditation compliance (skills focus on individual lessons/chapters, not program-level design).
- Student-facing features (skills are educator/author tools; they do not interact directly with learners).
- Version control or content management beyond standard git workflows.
- Analytics, telemetry, or learning analytics dashboards.

## Scope Clarifications

This feature builds **Claude Code skills** (model-invoked capabilities) rather than standalone applications or user-invoked commands. Skills are automatically activated by Claude when context matches their descriptions, following progressive disclosure patterns to minimize context usage.

### Six Core Skills (P1 Implementation)

Each skill addresses one focused capability in Python educational content development:

1. **learning-objectives.md**: Generates measurable learning outcomes aligned with Bloom's taxonomy, identifies prerequisites, and maps to assessment methods.

2. **concept-scaffolding.md**: Breaks down complex Python concepts (OOP, async, decorators) into progressive learning steps with cognitive load management.

3. **code-example-generator.md**: Creates runnable, pedagogically sound code examples that demonstrate one concept clearly with appropriate comments and progression.

4. **exercise-designer.md**: Designs varied practice activities applying evidence-based strategies (retrieval practice, spaced repetition, interleaving, desirable difficulty).

5. **technical-clarity.md**: Reviews technical explanations for jargon, clarity, completeness, and readability; suggests improvements for target audience level.

6. **assessment-builder.md**: Creates assessments (MCQ, code-completion, projects) with meaningful distractors, rubrics, and alignment to learning objectives.

### Optional Enhancement Skills (P2 Implementation)

7. **book-architecture.md**: Overall book structure, chapter flow, balancing tutorial progression with reference accessibility (for comprehensive book projects).

8. **ai-augmented-teaching.md**: Designs learning experiences that prepare students for AI-assisted development workflows (prompt engineering, AI pair programming).

### Skill Architecture Principles

- **Model-invoked**: Claude autonomously activates skills based on educator context and needs.
- **Progressive disclosure**: Supporting files loaded only when relevant (unbounded context through layered file loading).
- **Independent but composable**: Each skill works standalone; skills MAY consume outputs from other skills when available.
- **Python-focused, broadly applicable**: While optimized for Python pedagogy, skills are designed to adapt to other programming languages with minimal changes.
