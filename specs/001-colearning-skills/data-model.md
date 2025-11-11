# Data Model: Colearning Skills

**Feature**: 001-colearning-skills
**Date**: 2025-10-28
**Purpose**: Define skill structures, entities, and data flows

---

## Core Entities

### 1. Skill Metadata (Layer 1 - Loaded at Startup)

**Purpose**: Enable Claude to determine when to activate each skill

```yaml
name: string  # lowercase-with-hyphens, max 64 chars
description: string  # activation triggers, max 1024 chars
allowed-tools: List[ToolName]  # optional: Read, Grep, Glob, Bash, Edit, Write
```

**Validation Rules:**
- `name` must match `^[a-z0-9-]{1,64}$`
- `description` must be 50-1024 characters (minimum for meaningful context)
- `description` must include specific trigger terms, use cases, domain terminology
- `allowed-tools` if present must only contain valid tool names

**Example:**
```yaml
name: learning-objectives
description: |
  Generate measurable learning outcomes when educator requests "define objectives",
  "create learning goals", or "identify prerequisites" for Python concepts.
  Applies Bloom's taxonomy levels (Remember, Understand, Apply, Analyze,
  Evaluate, Create) and maps objectives to assessment methods.
```

---

### 2. Skill Instructions (Layer 2 - Loaded on Activation)

**Purpose**: Provide detailed guidance for skill execution

**Structure** (SKILL.md):
```markdown
---
[Frontmatter from Layer 1]
---

## Purpose

[What this skill does and when to use it - 2-3 sentences]

## When to Activate

[Specific scenarios that trigger this skill - bulleted list]

## Inputs

[What information/content the skill expects]

## Process

[Step-by-step instructions for execution]

## Output Format

[Structure of the skill's response]

## Examples

[2-3 concrete examples showing input → process → output]

## References

[Pointers to supporting files in reference/ directory]
```

**Validation Rules:**
- Must include all sections (no optional sections in core SKILL.md)
- Process steps must be numbered and actionable
- Examples must be complete (show full input/output)
- References must point to existing files

---

### 3. Supporting Resources (Layer 3 - Loaded on Reference)

#### 3.1 Reference Documentation (`reference/`)

**Purpose**: Provide foundational knowledge loaded when SKILL.md references it

**File Naming**: `concept-name.md` (lowercase, descriptive)

**Structure**:
```markdown
# [Concept Name]

## Overview
[2-3 sentence summary]

## Key Principles
[Bulleted list of core ideas]

## Application to Python Education
[How this applies to our use case]

## Examples
[Concrete examples]

## Common Pitfalls
[What to avoid]

## Further Reading
[Optional: external resources]
```

**Examples**:
- `reference/blooms-taxonomy-programming.md`
- `reference/cognitive-load-theory.md`
- `reference/python-best-practices.md`
- `reference/scaffolding-patterns.md`

#### 3.2 Templates (`templates/`)

**Purpose**: Reusable output structures

**Formats**: Markdown (`.md`) or YAML (`.yml`)

**Naming**: `output-type-template.{md|yml}`

**Example** (`templates/learning-objective-template.yml`):
```yaml
objective:
  level: ""  # Bloom's level: Remember|Understand|Apply|Analyze|Evaluate|Create
  statement: ""  # Measurable outcome: "Student will be able to..."
  prerequisites:
    - ""  # List of prior knowledge required
  assessment_methods:
    - type: ""  # formative|summative|performance
      description: ""
  cognitive_load: ""  # low|medium|high
```

#### 3.3 Scripts (`scripts/`)

**Purpose**: Executable validation, analysis, or generation utilities

**Language**: Python 3.13+ with type hints

**Naming**: `verb-noun.py` (e.g., `validate-syntax.py`, `analyze-readability.py`)

**Structure**:
```python
"""
Script purpose: [one line description]

Usage: python scripts/script-name.py [args]

Returns: Exit code 0 on success, non-zero on failure
"""

from typing import TypeAlias
from pathlib import Path

# Type aliases for clarity
CodeExample: TypeAlias = str
ValidationResult: TypeAlias = tuple[bool, str]


def main_function(input_data: InputType) -> OutputType:
    """
    Core functionality with type hints.

    Args:
        input_data: Description

    Returns:
        Description of return value

    Raises:
        SpecificError: When this happens
    """
    pass


if __name__ == "__main__":
    # CLI entry point
    import sys
    # Handle arguments, call main_function, exit with appropriate code
```

**Validation Rules:**
- All functions must have type hints
- Must include docstrings (module, function, class)
- Must handle errors gracefully (no uncaught exceptions)
- Must exit with code 0 (success) or 1+ (failure)

---

## Skill-Specific Data Models

### Learning-Objectives Skill

**Output Entity**: `LearningObjective`

```yaml
objectives:
  - id: string  # unique within lesson, e.g., "obj-001"
    level: BloomsLevel  # Remember|Understand|Apply|Analyze|Evaluate|Create
    statement: string  # "Student will be able to [measurable action]"
    prerequisites:
      - concept: string  # Prerequisite concept name
        chapter: string|null  # Where it's taught (or null if assumed)
    assessment_methods:
      - type: AssessmentType  # formative|summative|performance
        description: string
        alignment: string  # How it measures the objective
    cognitive_complexity: CognitiveLoad  # low|medium|high
    estimated_time_minutes: int  # Time to achieve mastery
```

**Validation**:
- `statement` must start with action verb (Bloom's verb list)
- `prerequisites` must be validated against prior content
- At least one `assessment_method` required
- `cognitive_complexity` derived from Bloom's level + prerequisites count

---

### Concept-Scaffolding Skill

**Output Entity**: `ScaffoldingPlan`

```yaml
concept: string  # Target complex concept
complexity_rating: int  # 1-10 scale
scaffolding_steps:
  - step_number: int  # 1 to N (max 7 per CLT research)
    title: string  # Brief step name
    description: string  # What's introduced in this step
    cognitive_load: CognitiveLoad  # low|medium|high
    new_concepts_introduced:
      - string  # List of new ideas (max 3 per step per CLT)
    builds_on:
      - int  # References to prior step numbers
    worked_example: string|null  # Optional: reference to example file
    checkpoint: string|null  # Optional: how to verify understanding
    support_strategies:
      - string  # Scaffolding aids (hints, analogies, visuals)
total_cognitive_load: CognitiveLoad  # Overall difficulty assessment
recommended_duration_minutes: int  # Total teaching time
```

**Validation**:
- Max 7 `scaffolding_steps` (CLT working memory limit)
- Each step max 3 `new_concepts_introduced`
- `builds_on` must reference valid prior steps (no forward references)
- `cognitive_load` escalates gradually (can't jump low→high)

---

### Code-Example-Generator Skill

**Output Entity**: `CodeExample`

```yaml
concept: string  # What this example demonstrates
difficulty: Difficulty  # beginner|intermediate|advanced
code: string  # Actual Python code (will be validated)
explanation:
  what: string  # High-level: what does this code do?
  how: string  # Step-by-step: how does it work?
  why: string  # Design rationale: why this approach?
comments_inline: bool  # true if code includes inline comments
demonstrates:
  - string  # List of concepts shown in this example
avoids:
  - string  # Concepts intentionally NOT included (focus)
validation:
  syntax_valid: bool  # Passed AST parsing
  executed_successfully: bool  # Ran without errors
  output: string|null  # Expected output if applicable
  execution_time_ms: int|null  # How long it took
style_compliance:
  pep8: bool  # Passed PEP 8 checks
  type_hints: bool  # All functions have type hints
  docstrings: bool  # Functions have docstrings
```

**Validation**:
- `code` must pass syntax validation (AST parse)
- `code` must execute successfully in sandbox (subprocess)
- `demonstrates` must list 1-3 concepts (focused examples)
- All `style_compliance` checks must pass

---

### Exercise-Designer Skill

**Output Entity**: `ExerciseSet`

```yaml
learning_objectives:
  - string  # References to objectives this exercise addresses
exercise_type: ExerciseType  # fill-in-blank|debug-this|build-from-scratch|extend-code|quiz
difficulty_progression: List[Difficulty]  # [beginner, beginner, intermediate, ...]
exercises:
  - id: string  # e.g., "ex-001"
    type: ExerciseType
    difficulty: Difficulty
    prompt: string  # What student should do
    starter_code: string|null  # Provided code (if applicable)
    solution: string  # Reference solution
    test_cases:
      - input: string
        expected_output: string
        reasoning: string  # Why this test case matters
    learning_strategies:
      - StrategyType  # retrieval-practice|spaced-repetition|interleaving|desirable-difficulty
    estimated_time_minutes: int
    hints:
      - level: int  # 1=light hint, 2=moderate, 3=heavy
        text: string
    rubric:
      - criterion: string
        points: int
        description: string
```

**Validation**:
- At least 3 exercises per set
- `difficulty_progression` must be gradual (no sudden jumps)
- Each exercise must identify at least 1 `learning_strategy`
- `test_cases` must cover normal, edge, and error cases
- `solution` must pass all `test_cases`

---

### Technical-Clarity Skill

**Output Entity**: `ClarityReport`

```yaml
analyzed_text: string  # Original content (or reference)
readability_metrics:
  flesch_kincaid_grade: float  # Grade level
  avg_sentence_length: int
  complex_word_percentage: float
target_audience: Audience  # beginner|intermediate|advanced
issues_identified:
  - line_number: int|null  # Location if applicable
    category: IssueCategory  # jargon|unclear-passage|missing-context|weak-analogy
    severity: Severity  # low|medium|high
    finding: string  # What's wrong
    suggestion: string  # How to fix it
    example: string|null  # Improved version
completeness_check:
  core_concepts_covered: List[string]
  missing_critical_info: List[string]
  depth_assessment: DepthLevel  # surface|adequate|comprehensive
analogies_reviewed:
  - original_analogy: string
    effectiveness: EffectivenessRating  # poor|adequate|excellent
    suggestion: string|null  # If poor/adequate
code_snippets_validated:
  - snippet: string
    syntactically_correct: bool
    supports_explanation: bool
    issue: string|null
```

**Validation**:
- `readability_metrics` must be calculated programmatically
- `issues_identified` must be actionable (include suggestions)
- All `code_snippets` must be syntax-checked

---

### Assessment-Builder Skill

**Output Entity**: `AssessmentPackage`

```yaml
learning_objectives_assessed:
  - string  # Which objectives this assessment covers
question_distribution:
  recall: int  # Count of memorization questions
  conceptual: int  # Count of understanding questions
  application: int  # Count of problem-solving questions
  synthesis: int  # Count of design/creation questions
questions:
  - id: string  # e.g., "q-001"
    type: QuestionType  # multiple-choice|code-completion|debugging|short-answer|project
    cognitive_level: BloomsLevel
    objective_id: string  # Links to learning objective
    prompt: string  # The question
    # Type-specific fields:
    # For multiple-choice:
    options:
      - id: string  # A, B, C, D
        text: string
        is_correct: bool
        rationale: string  # Why correct/incorrect
        misconception: string|null  # If distractor, what misconception it targets
    # For code-completion/debugging:
    starter_code: string
    solution: string
    test_cases: List[TestCase]
    # For short-answer/project:
    rubric:
      - criterion: string
        points: int
        excellent: string  # Descriptor for full points
        adequate: string  # Descriptor for partial
        insufficient: string  # Descriptor for low/zero
    estimated_time_minutes: int
    difficulty: Difficulty
validation:
  objectives_coverage: float  # Percentage of objectives assessed
  cognitive_distribution_balanced: bool  # Not all recall questions
  distractors_meaningful: bool  # MCQs have misconception-based wrong answers
```

**Validation**:
- Must assess at least 80% of learning objectives
- `cognitive_distribution` must include 60%+ non-recall questions
- MCQ `distractors` must target common misconceptions (not random)
- All code must be validated via sandbox

---

## Data Flow Diagrams

### Skill Activation Flow

```
User Request
    ↓
Claude analyzes context
    ↓
Layer 1: Scans all skill descriptions (metadata)
    ↓
Match found? → Yes → Activate skill
                 ↓
                Layer 2: Load SKILL.md (instructions)
                 ↓
                Execute skill process
                 ↓
                Layer 3: Load referenced resources (on-demand)
                 ↓
                Generate output entity
                 ↓
                Validate output
                 ↓
                Return to user
    ↓
No match? → Continue with general Claude response
```

### Sequential Multi-Skill Flow

```
User Request: "Generate code example and review for clarity"
    ↓
Claude detects 2 distinct purposes
    ↓
Skill 1: code-example-generator activates
    ↓
    Generates CodeExample entity
    ↓
    Validates syntax + execution
    ↓
    Outputs code example
    ↓
Skill 2: technical-clarity activates
    ↓
    Receives CodeExample as input
    ↓
    Generates ClarityReport entity
    ↓
    Outputs clarity analysis
    ↓
Combined output presented to user:
    [Code Example]
    [Clarity Analysis]
```

---

## File System Structure

```
.claude/skills/
├── learning-objectives/
│   ├── SKILL.md                          # Layer 2
│   ├── reference/                         # Layer 3
│   │   ├── blooms-taxonomy-programming.md
│   │   └── prerequisite-analysis.md
│   ├── templates/                         # Layer 3
│   │   └── learning-objective-template.yml
│   └── scripts/                           # Layer 3
│       └── validate-objectives.py
├── concept-scaffolding/
│   ├── SKILL.md
│   ├── reference/
│   │   ├── cognitive-load-theory.md
│   │   └── scaffolding-strategies.md
│   ├── templates/
│   │   └── scaffolding-plan-template.yml
│   └── scripts/
│       └── assess-cognitive-load.py
├── code-example-generator/
│   ├── SKILL.md
│   ├── reference/
│   │   ├── python-best-practices.md
│   │   └── example-patterns.md
│   ├── templates/
│   │   └── code-example-template.md
│   └── scripts/
│       ├── validate-syntax.py
│       └── execute-sandbox.py
├── exercise-designer/
│   ├── SKILL.md
│   ├── reference/
│   │   ├── exercise-types.md
│   │   ├── evidence-based-strategies.md
│   │   └── difficulty-progression.md
│   ├── templates/
│   │   ├── exercise-template.yml
│   │   └── rubric-template.yml
│   └── scripts/
│       └── generate-test-cases.py
├── technical-clarity/
│   ├── SKILL.md
│   ├── reference/
│   │   ├── readability-metrics.md
│   │   ├── analogy-patterns.md
│   │   └── clarity-checklist.md
│   ├── templates/
│   │   └── clarity-report-template.yml
│   └── scripts/
│       ├── analyze-readability.py
│       └── check-completeness.py
└── assessment-builder/
    ├── SKILL.md
    ├── reference/
    │   ├── question-types.md
    │   ├── distractor-design.md
    │   └── rubric-guidelines.md
    ├── templates/
    │   ├── assessment-template.yml
    │   └── rubric-template.yml
    └── scripts/
        └── validate-assessment.py
```

---

## Validation & Error Handling

### Validation Points

1. **Skill Metadata**: Name format, description length, tool restrictions
2. **SKILL.md Structure**: All required sections present, examples complete
3. **Output Entities**: Schema validation per data model
4. **Code**: Syntax + execution validation via sandbox
5. **References**: All file paths resolve to existing resources

### Error Handling Strategy (Per Clarification #1: Hard Failure)

**When Validation Fails:**
```yaml
error:
  skill_name: string
  stage: ActivationStage  # metadata|load|execution|validation
  error_type: ErrorType  # missing-file|validation-failure|sandbox-error
  message: string  # Clear description
  suggested_fix: string  # How to resolve
  user_action_required: true  # Always true per clarification
```

**Example Error**:
```yaml
error:
  skill_name: "code-example-generator"
  stage: "execution"
  error_type: "sandbox-error"
  message: "Generated code exceeded 5s execution timeout"
  suggested_fix: "Simplify the example to reduce complexity, or check for infinite loops"
  user_action_required: true
```

---

## Performance Considerations

**Target Metrics** (from Success Criteria SC-005 to SC-010):

| Skill | Time Target | Validation Method |
|-------|-------------|-------------------|
| learning-objectives | <30s | Generate 5 objectives for typical topic |
| concept-scaffolding | <45s | Generate 5-step breakdown |
| code-example-generator | <20s | Generate + validate 1 example |
| exercise-designer | <60s | Generate 3-5 exercises |
| technical-clarity | <30s | Analyze 500-word explanation |
| assessment-builder | <60s | Generate 5-question assessment |

**Optimization Strategies:**
- Progressive disclosure minimizes context load
- Scripts pre-validate before returning to skill
- Caching of frequently-used reference files (if supported)
- Parallel validation where possible (syntax + style checks)

---

**Data Model Complete** ✅
**Next**: Create skill templates and contracts (YAML schemas)
