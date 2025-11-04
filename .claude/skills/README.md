# CoLearning Python Skills

This directory contains specialized pedagogical skills for creating high-quality Python programming education content. These skills are designed to be **semantically activated** based on the educator's intent and teaching needs, using Claude Code's natural language understanding.


## Architecture: Progressive Disclosure

The skills follow a **progressive disclosure architecture** with three layers:

### Layer 1: Skill Description (SKILL.md frontmatter)
Each skill's description in SKILL.md provides a concise overview that Claude uses for semantic activation. When an educator's request matches the skill's purpose, Claude automatically activates the appropriate skill.

### Layer 2: Process and Guidance (SKILL.md body)
Once activated, the skill provides step-by-step guidance, explaining:
- When to use the skill
- What inputs are required
- How to execute the process
- What outputs to generate
- Common patterns and troubleshooting

### Layer 3: Deep Reference Materials
For complex concepts, skills reference detailed documents:
- `reference/`: In-depth guides on pedagogical theories, strategies, and patterns
- `templates/`: Structured formats for outputs (YAML, Markdown)
- `scripts/`: Validation and assessment utilities

This layered approach ensures Claude can:
1. **Quickly determine relevance** (Layer 1 descriptions)
2. **Execute effectively** (Layer 2 processes)
3. **Apply rigor** (Layer 3 references and validation)

---

## How Skills Are Activated

Skills are **semantically activated** based on the intent and context of your request. You don't need to explicitly invoke a skill name - Claude understands your pedagogical goal and activates the appropriate skill automatically.

### Activation Factors

1. **Intent Keywords**: Words like "objectives", "scaffold", "examples", "exercises", "assessment", "clarity", "structure", "AI integration"

2. **Task Context**: What you're trying to accomplish (planning, creating, reviewing, validating)

3. **Teaching Stage**: Curriculum design, content creation, quality review, or student support

4. **Pedagogical Concerns**: Cognitive load, measurability, accessibility, evidence-based strategies

### Example Activation Patterns

**Request**: "I need to teach Python decorators to intermediate learners. How should I structure this?"

**Skills Activated**:
1. `concept-scaffolding` (primary) - Breaks down complex concept into steps
2. `learning-objectives` (supporting) - Defines what students should achieve
3. `code-example-generator` (supporting) - Creates demonstration examples

---

**Request**: "Create a quiz for my functions lesson that tests understanding, not just memorization."

**Skills Activated**:
1. `assessment-builder` (primary) - Creates balanced assessment
2. `learning-objectives` (supporting) - Aligns questions to objectives

---

**Request**: "Is this tutorial clear for beginners? I'm worried about jargon."

**Skills Activated**:
1. `technical-clarity` (primary) - Reviews readability and accessibility

---

**Request**: "Design practice exercises for loops that also review conditionals from last week."

**Skills Activated**:
1. `exercise-designer` (primary) - Creates exercises with spaced repetition

---

## Usage Principles

### 1. Skills Are Complementary
Multiple skills often work together. For example, after generating learning objectives, you might:
- Use `concept-scaffolding` to break down complex objectives
- Use `code-example-generator` to create demonstrations
- Use `exercise-designer` to create practice activities
- Use `assessment-builder` to measure achievement

### 2. Progressive Workflow
Skills support a natural teaching workflow:
1. **Plan**: `learning-objectives`, `book-architecture`
2. **Create**: `concept-scaffolding`, `code-example-generator`, `exercise-designer`
3. **Assess**: `assessment-builder`
4. **Review**: `technical-clarity`
5. **Integrate AI**: `ai-augmented-teaching`

### 3. Validation and Iteration
Most skills include validation scripts in their `scripts/` directories. Use these to:
- Check quality objectively
- Identify specific improvements
- Ensure standards are met

### 4. Reference Materials
Each skill's `reference/` directory contains deep pedagogical knowledge:
- Bloom's Taxonomy for programming
- Cognitive Load Theory
- Evidence-based learning strategies
- Best practices and patterns

You don't need to read these directly - skills reference them automatically when needed. But they're available if you want to understand the underlying pedagogy.

## Skills Directory Structure

```
.claude/skills/
├── README.md                          # This file
├── _shared/                          # Shared utilities
│   ├── sandbox-executor.py           # Safe code execution
│   └── readability-analyzer.py       # Text analysis
│
├── learning-objectives/
│   ├── SKILL.md                      # Skill definition and process
│   ├── reference/                    # Pedagogical guides
│   │   ├── blooms-taxonomy-programming.md
│   │   ├── prerequisite-analysis.md
│   │   └── assessment-methods.md
│   ├── templates/                    # Output formats
│   │   └── learning-objective-template.yml
│   └── scripts/                      # Validation tools
│       └── validate-objectives.py
│
├── concept-scaffolding/
│   ├── SKILL.md
│   ├── reference/
│   │   ├── cognitive-load-theory.md
│   │   ├── scaffolding-strategies.md
│   │   └── worked-examples.md
│   ├── templates/
│   │   └── scaffolding-plan-template.yml
│   └── scripts/
│       └── assess-cognitive-load.py
│
├── code-example-generator/
│   ├── SKILL.md
│   ├── reference/
│   │   ├── python-best-practices.md
│   │   ├── example-patterns.md
│   │   └── pep8-summary.md
│   ├── templates/
│   │   └── code-example-template.md
│   └── scripts/
│       └── validate-syntax.py
│
├── exercise-designer/
│   ├── SKILL.md
│   ├── reference/
│   │   ├── exercise-types.md
│   │   ├── evidence-based-strategies.md
│   │   ├── difficulty-progression.md
│   │   └── spaced-repetition.md
│   ├── templates/
│   │   ├── exercise-template.yml
│   │   └── rubric-template.yml
│   └── scripts/
│       └── generate-test-cases.py
│
├── assessment-builder/
│   ├── SKILL.md
│   ├── reference/
│   │   ├── question-types.md
│   │   ├── distractor-design.md
│   │   ├── blooms-assessment-alignment.md
│   │   └── rubric-guidelines.md
│   ├── templates/
│   │   ├── assessment-template.yml
│   │   └── rubric-template.yml
│   └── scripts/
│       └── validate-assessment.py
│
├── technical-clarity/
│   ├── SKILL.md
│   ├── reference/
│   │   ├── readability-metrics.md
│   │   ├── gatekeeping-language.md
│   │   ├── analogy-patterns.md
│   │   └── clarity-checklist.md
│   ├── templates/
│   │   └── clarity-report-template.yml
│   └── scripts/
│       └── check-completeness.py
│
├── book-architecture/
│   ├── SKILL.md
│   ├── reference/
│   │   ├── structural-patterns.md
│   │   ├── chapter-dependencies.md
│   │   ├── chapter-flow-patterns.md
│   │   └── content-organization.md
│   ├── templates/
│   │   ├── book-outline-template.yml
│   │   └── chapter-structure-template.yml
│   └── scripts/
│       ├── validate-structure.py
│       └── analyze-flow.py
│
└── ai-augmented-teaching/
    ├── SKILL.md
    ├── reference/
    │   ├── prompt-engineering-pedagogy.md
    │   ├── ai-pair-programming-patterns.md
    │   ├── ai-tool-literacy.md
    │   └── ethical-ai-use.md
    ├── templates/
    │   ├── ai-lesson-template.yml
    │   └── prompt-design-template.md
    └── scripts/
        ├── assess-ai-integration.py
        └── validate-prompts.py
```

## Pedagogical Foundations

These skills are grounded in evidence-based teaching research:

- **Bloom's Taxonomy**: Cognitive levels from Remember to Create
- **Cognitive Load Theory**: Managing working memory limitations
- **Deliberate Practice**: Structured, purposeful skill development
- **Retrieval Practice**: Strengthening memory through recall
- **Spaced Repetition**: Distributing practice over time
- **Interleaving**: Mixing concepts to improve transfer
- **Worked Examples**: Demonstrating problem-solving processes
- **Scaffolding**: Progressive support reduction
- **Constructive Alignment**: Aligning objectives, activities, and assessment

## Quality Standards

All skills enforce quality through validation:

- **Measurability**: Objectives use action verbs and success criteria
- **Cognitive Balance**: Assessments test application, not just recall (60%+ non-recall)
- **Accessibility**: Content readability matches target audience
- **Evidence-Based**: Exercises apply cognitive science principles
- **Completeness**: Content includes prerequisites, examples, context
- **Runnable Code**: Examples are syntactically correct and executable
- **Ethical AI Use**: AI integration balances assistance with learning


## The Core Skills

### 1. learning-objectives
**Purpose**: Generate measurable learning outcomes aligned with Bloom's taxonomy

**When to use**:
- Planning curriculum or lesson design
- Creating assessments aligned to clear objectives
- Ensuring objectives are specific and testable
- Sequencing learning from basic recall to creative synthesis

**Key features**:
- Maps objectives to Bloom's 6 cognitive levels (Remember through Create)
- Identifies prerequisites and scaffolds learning progressively
- Validates measurability with action verbs
- Pairs objectives with appropriate assessment methods

**Activation examples**:
- "Create learning objectives for teaching Python decorators"
- "What should students achieve after learning about loops?"
- "Help me define measurable outcomes for my OOP unit"

---

### 2. concept-scaffolding
**Purpose**: Break down complex concepts into progressive learning steps with cognitive load management

**When to use**:
- Teaching advanced topics incrementally (OOP, decorators, async/await, metaclasses)
- Learners struggling with cognitive overload
- Designing progressions from simple to complex understanding
- Creating worked examples with fading support

**Key features**:
- Applies Cognitive Load Theory (max 2-3 new concepts per step for beginners)
- Creates 3-7 step learning progressions
- Provides worked examples for each step
- Includes verification checkpoints
- Assesses cognitive load to prevent overwhelm

**Activation examples**:
- "How do I teach Python decorators incrementally?"
- "Break down async/await for beginners"
- "Scaffold object-oriented programming concepts"

---

### 3. code-example-generator
**Purpose**: Generate runnable, pedagogically sound Python code examples with validation

**When to use**:
- Authors need teaching examples demonstrating specific concepts
- Creating progressive example sequences (simple to complex)
- Validating existing code examples for best practices
- Need examples that follow Python conventions (PEP 8)

**Key features**:
- Creates runnable, self-contained examples
- Progressive complexity (simple, realistic, complex)
- Validates syntax and executes in sandbox
- Includes clear explanations of what, how, and why
- Shows expected output and common mistakes

**Activation examples**:
- "Generate a beginner example for list comprehensions"
- "Show me a realistic example of exception handling"
- "Create code examples demonstrating dictionary methods"

---

### 4. exercise-designer
**Purpose**: Design deliberate practice exercises with evidence-based learning strategies

**When to use**:
- Creating homework assignments or problem sets
- Applying evidence-based strategies (retrieval practice, spaced repetition, interleaving)
- Establishing difficulty progression
- Generating varied exercise types (not just "write code from scratch")

**Key features**:
- 8 exercise types: fill-in-blank, debug-this, build-from-scratch, extend-code, trace-execution, explain-code, refactor, Parsons problems
- Applies spaced repetition (reviews prior concepts)
- Interleaves concepts (60% current, 30% recent, 10% older)
- Generates comprehensive test cases (normal, edge, error)
- Provides rubrics and progressive hints

**Activation examples**:
- "Create 5 exercises for practicing list methods"
- "Design exercises for loops that review conditionals"
- "Generate a problem set with varied difficulty"

---

### 5. assessment-builder
**Purpose**: Create balanced assessments with varied question types and meaningful distractors

**When to use**:
- Designing quizzes, tests, or exams
- Need questions at appropriate cognitive levels (60%+ non-recall)
- Creating MCQs with diagnostic distractors based on misconceptions
- Generating rubrics for open-ended questions

**Key features**:
- 10 question types: MCQ, code-completion, code-tracing, debugging, code-writing, explanation, code-review, prediction, comparison, project
- Creates meaningful MCQ distractors testing specific misconceptions
- Validates cognitive distribution (Remember through Create)
- Generates analytic rubrics for open-ended questions
- Provides complete answer keys with explanations

**Activation examples**:
- "Create a balanced assessment for Python functions"
- "Design a quiz testing application skills, not just recall"
- "Add meaningful distractors to these MCQs"

---

### 6. technical-clarity
**Purpose**: Review technical explanations for jargon, readability, and accessibility

**When to use**:
- Reviewing draft tutorials, documentation, or book chapters
- Identifying gatekeeping language ("obviously", "simply", etc.)
- Assessing readability level vs. target audience
- Checking for completeness (prerequisites, context, examples)

**Key features**:
- Calculates readability metrics (Flesch-Kincaid grade level)
- Identifies jargon and checks definition status
- Flags gatekeeping language with replacements
- Assesses completeness (prerequisites, examples, context, motivation)
- Provides prioritized recommendations (critical/important/enhancement)

**Activation examples**:
- "Review this Python tutorial for clarity"
- "Is this explanation appropriate for beginners?"
- "Check this content for gatekeeping language"

---


### 7. book-scaffolding

**Purpose**: Structure book content with logical chapter flow and dependency management

**When to use**:
- Planning book structure or course curriculum
- Organizing chapters with prerequisite dependencies
- Ensuring logical concept progression
- Balancing chapter lengths and complexity

**Key features**:
- Validates chapter dependency chains
- Checks for structural patterns (foundation, progression, integration, specialization)
- Analyzes chapter flow and concept scaffolding
- Identifies circular dependencies
- Balances chapter lengths and difficulty

**Activation examples**:
- "Plan the structure for my Python book"
- "Validate these chapter dependencies"
- "What order should I teach these concepts?"

---


### 8. ai-augmented-teaching

**Purpose**: Design learning experiences for AI-assisted software development

### 9. content-evaluation-framework
**Purpose**: Evaluate content quality using a structured rubric (technical accuracy, pedagogy, writing, structure, AI-first teaching, constitution compliance) with weighted scoring and actionable feedback

**When to use**:
- Integrating AI tools (ChatGPT, GitHub Copilot, Claude) into programming curriculum
- Teaching prompt engineering as a core skill
- Balancing AI assistance with foundational learning
- Establishing ethical guidelines for AI use
- Assessing AI integration appropriateness

**Key features**:
- 5 AI pair programming patterns (Explainer, Debugger, Code Reviewer, Pair Programmer, Hypothesis Validator)
- Teaches prompt engineering pedagogy (context, constraints, iteration)
- Builds AI tool literacy (capabilities, limitations, verification)
- Establishes ethical frameworks (disclosure, attribution, understanding requirements)
- Assesses balance (40% foundation / 40% AI-assisted / 20% verification)

**Activation examples**:
- "How do I integrate AI tools into my Python course?"
- "Design a lesson balancing AI assistance with independent learning"
- "Create a prompt engineering curriculum"


## Contributing

To add or modify skills:

1. Follow the three-layer progressive disclosure architecture
2. Write clear, specific semantic descriptions for activation
3. Provide step-by-step processes with examples
4. Include validation scripts where possible
5. Ground approach in pedagogical research
6. Test activation with varied natural language requests

## License

These skills are part of the CoLearning Python project.

## Support

For questions or issues, refer to the project documentation or raise an issue in the repository.
