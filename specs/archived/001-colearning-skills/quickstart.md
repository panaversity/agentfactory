# Quickstart: Colearning Skills for Python Book Agent

**Feature**: 001-colearning-skills
**For**: Educators and content creators building Python learning materials
**Goal**: Install and use 6 specialized Claude Code skills for educational content development

---

## Prerequisites

- **Claude Code**: Installed and working (version with model-invoked skills support)
- **Python**: 3.13+ installed and accessible via `python3` command
- **Skills Directory**: `.claude/skills/` (created automatically)

---

## Installation

### Step 1: Clone or Copy Skills

```bash
# Navigate to your project
cd /path/to/colearning-python

# Skills will be located in .claude/skills/ (created during implementation phase)
```

### Step 2: Verify Installation

Claude automatically loads skills from `.claude/skills/` at startup. To verify:

```bash
# Check that skill directories exist
ls -la .claude/skills/

# Expected output (after implementation):
# learning-objectives/
# concept-scaffolding/
# code-example-generator/
# exercise-designer/
# technical-clarity/
# assessment-builder/
```

---

## Quick Usage Guide

### Skill 1: learning-objectives

**Purpose**: Generate measurable learning outcomes aligned with Bloom's taxonomy

**When to use**: Planning a lesson or chapter; need to define what students will learn

**Example interaction**:

```
You: "Define learning objectives for teaching Python decorators"

Claude activates learning-objectives skill →

Output:
---
objectives:
  - id: obj-001
    level: Understand
    statement: "Student will be able to explain what decorators do and when to use them"
    prerequisites:
      - concept: "functions"
        chapter: "Chapter 5"
      - concept: "first-class functions"
        chapter: "Chapter 8"
    assessment_methods:
      - type: formative
        description: "Verbal explanation of decorator purpose"
        alignment: "Tests conceptual understanding"
    cognitive_complexity: medium
    estimated_time_minutes: 45

  - id: obj-002
    level: Apply
    statement: "Student will be able to write simple decorators for logging function calls"
    ...
---
```

---

### Skill 2: concept-scaffolding

**Purpose**: Break down complex concepts into learnable steps with cognitive load management

**When to use**: Planning how to teach difficult topics; need progressive introduction strategy

**Example interaction**:

```
You: "How should I scaffold teaching async/await in Python?"

Claude activates concept-scaffolding skill →

Output:
---
concept: "Asynchronous Programming with async/await"
complexity_rating: 8
scaffolding_steps:
  - step_number: 1
    title: "Synchronous baseline"
    description: "Show familiar synchronous code with I/O operations"
    cognitive_load: low
    new_concepts_introduced: ["blocking I/O"]
    builds_on: []
    ...

  - step_number: 2
    title: "The waiting problem"
    description: "Demonstrate performance issues with sequential I/O"
    cognitive_load: medium
    new_concepts_introduced: ["concurrency need", "event loop concept"]
    builds_on: [1]
    ...
---
```

---

### Skill 3: code-example-generator

**Purpose**: Create runnable, pedagogically sound code examples validated via sandbox execution

**When to use**: Need working code examples for lessons; want syntax-validated and tested code

**Example interaction**:

```
You: "Generate a beginner code example for list comprehensions"

Claude activates code-example-generator skill →

Output:
---
concept: "list comprehensions"
difficulty: beginner
code: |
  # Traditional loop approach
  squares: list[int] = []
  for x in range(10):
      squares.append(x ** 2)

  # List comprehension (same result, more concise)
  squares_comp: list[int] = [x ** 2 for x in range(10)]

  print(f"Loop result: {squares}")
  print(f"Comprehension result: {squares_comp}")

explanation:
  what: "Demonstrates two ways to create a list of squared numbers"
  how: "First shows explicit loop with append, then equivalent comprehension"
  why: "Comprehensions are more Pythonic and often more readable for simple transformations"

demonstrates: ["list comprehensions", "range function", "type hints"]
validation:
  syntax_valid: true
  executed_successfully: true
  output: |
    Loop result: [0, 1, 4, 9, 16, 25, 36, 49, 64, 81]
    Comprehension result: [0, 1, 4, 9, 16, 25, 36, 49, 64, 81]
  execution_time_ms: 12
---
```

---

### Skill 4: exercise-designer

**Purpose**: Design deliberate practice exercises applying evidence-based learning strategies

**When to use**: Creating practice problems; need varied exercise types with progressive difficulty

**Example interaction**:

```
You: "Create exercises for practicing functions in Python"

Claude activates exercise-designer skill →

Output:
---
learning_objectives: ["obj-005", "obj-006"]
exercise_type: "mixed"
difficulty_progression: [beginner, beginner, intermediate]
exercises:
  - id: "ex-001"
    type: "fill-in-blank"
    difficulty: beginner
    prompt: |
      Complete the function to calculate the area of a rectangle:

      def calculate_area(length: float, width: float) -> ____:
          '''Calculate rectangle area.'''
          return ____

    solution: |
      def calculate_area(length: float, width: float) -> float:
          '''Calculate rectangle area.'''
          return length * width

    learning_strategies: ["retrieval-practice"]
    estimated_time_minutes: 10
    hints:
      - level: 1
        text: "What type should the function return?"
      - level: 2
        text: "Area = length × width"
    ...
---
```

---

### Skill 5: technical-clarity

**Purpose**: Review technical explanations for jargon, clarity, completeness, and readability

**When to use**: Reviewing written explanations; checking if content is accessible to target audience

**Example interaction**:

```
You: "Review this explanation for clarity:

'Decorators are syntactic sugar for HOFs that modify callables via closures, leveraging Python's first-class function semantics.'"

Claude activates technical-clarity skill →

Output:
---
analyzed_text: "[original text]"
readability_metrics:
  flesch_kincaid_grade: 16.2
  avg_sentence_length: 15
  complex_word_percentage: 45.0
target_audience: beginner

issues_identified:
  - line_number: 1
    category: jargon
    severity: high
    finding: "Multiple undefined terms: 'syntactic sugar', 'HOFs', 'closures', 'first-class function semantics'"
    suggestion: "Define each term before using, or replace with simpler language"
    example: |
      Decorators are a Python feature that lets you modify functions. They work by wrapping
      one function around another, adding behavior without changing the original function's code.

  - line_number: 1
    category: missing-context
    severity: medium
    finding: "No explanation of WHY decorators are useful"
    suggestion: "Add practical use case (e.g., logging, timing, authentication)"
    ...
---
```

---

### Skill 6: assessment-builder

**Purpose**: Create assessments measuring deep understanding (not just memorization)

**When to use**: Building quizzes, tests, or projects; need questions aligned with learning objectives

**Example interaction**:

```
You: "Create an assessment for Python list methods"

Claude activates assessment-builder skill →

Output:
---
learning_objectives_assessed: ["obj-010", "obj-011", "obj-012"]
question_distribution:
  recall: 1
  conceptual: 3
  application: 2
  synthesis: 1

questions:
  - id: "q-001"
    type: "multiple-choice"
    cognitive_level: "Understand"
    objective_id: "obj-010"
    prompt: |
      Which list method would you use to add an element to the END of a list?
    options:
      - id: "A"
        text: "list.insert(0, element)"
        is_correct: false
        rationale: "insert(0, element) adds to the BEGINNING"
        misconception: "Confusion between index 0 (start) and end position"
      - id: "B"
        text: "list.append(element)"
        is_correct: true
        rationale: "append() adds to the end of the list"
      - id: "C"
        text: "list.extend([element])"
        is_correct: false
        rationale: "extend() works but is not idiomatic for single elements"
        misconception: "Confusing append vs extend use cases"
    ...
---
```

---

## Skill Activation Patterns

### Single Skill Activation

Claude detects ONE relevant skill and activates it:

```
You: "Define objectives for teaching classes"
→ learning-objectives activates
```

### Sequential Multi-Skill Activation

Claude detects multiple skills serving DIFFERENT purposes and runs them in sequence:

```
You: "Generate a code example for decorators and review it for clarity"
→ code-example-generator activates (generates example)
→ technical-clarity activates (reviews example)
→ Combined output presented
```

### How Claude Chooses Skills

Based on **Layer 1 metadata** (skill descriptions), Claude matches:
- Specific trigger phrases ("define objectives", "create example", "review for clarity")
- Domain terminology (Python concepts, pedagogical terms)
- Context clues (user role: educator, content creator)

---

## Customization

### Modifying Skill Behavior

Edit `.claude/skills/<skill-name>/SKILL.md` to adjust:
- **Description**: Change activation triggers
- **Process steps**: Modify how skill executes
- **Output format**: Adjust structure of responses

### Adding Reference Materials

Add new files to `.claude/skills/<skill-name>/reference/`:
- Create `new-concept.md` with relevant information
- Reference it in SKILL.md: "See reference/new-concept.md for details"
- Claude loads it automatically when referenced

### Creating Custom Scripts

Add validation or analysis scripts to `.claude/skills/<skill-name>/scripts/`:
1. Write Python 3.13+ script with type hints
2. Make executable: `chmod +x scripts/your-script.py`
3. Reference in SKILL.md process steps
4. Claude runs it via Bash tool when needed

---

## Troubleshooting

### Skill Not Activating

**Problem**: You request something but the skill doesn't activate

**Solutions**:
1. Use more specific language matching skill description triggers
2. Check `.claude/skills/<skill-name>/SKILL.md` description for trigger phrases
3. Explicitly mention the skill: "Use the learning-objectives skill to..."

### Validation Errors

**Problem**: Skill fails with validation error (hard failure per clarification #1)

**Example Error**:
```
error:
  skill_name: "code-example-generator"
  stage: "execution"
  error_type: "sandbox-error"
  message: "Generated code exceeded 5s execution timeout"
  suggested_fix: "Simplify the example to reduce complexity"
  user_action_required: true
```

**Solutions**:
- Follow the `suggested_fix` guidance
- Simplify your request
- Check supporting files are present and valid

### Multiple Skills Activating

**Problem**: Skills conflict or activate unexpectedly

**Solution**:
- Skills have mutually exclusive descriptions (by design)
- If unintended activation, report as bug - descriptions need refinement
- Explicitly request single skill: "Use ONLY code-example-generator..."

---

## Best Practices

### For Educators

1. **Start with learning-objectives**: Define what students will learn before creating content
2. **Use concept-scaffolding**: Plan complex topic introduction before writing
3. **Validate code examples**: Always use code-example-generator to ensure runnable code
4. **Review for clarity**: Run technical-clarity on explanations before publishing

### Workflow Example

```
1. Define objectives → learning-objectives skill
2. Plan scaffolding → concept-scaffolding skill
3. Create examples → code-example-generator skill
4. Review clarity → technical-clarity skill
5. Design exercises → exercise-designer skill
6. Build assessment → assessment-builder skill
```

### For Content Creators

- **Be specific**: "Generate beginner example for X" > "help with X"
- **Iterate**: Use skills multiple times, refining based on outputs
- **Combine skills**: Request multi-skill workflows ("generate and review")
- **Validate outputs**: Check that generated content meets your standards

---

## Performance Expectations

| Skill | Typical Response Time | Output Size |
|-------|----------------------|-------------|
| learning-objectives | <30 seconds | 3-5 objectives |
| concept-scaffolding | <45 seconds | 3-7 steps |
| code-example-generator | <20 seconds | 1 validated example |
| exercise-designer | <60 seconds | 3-5 exercises |
| technical-clarity | <30 seconds | Detailed analysis |
| assessment-builder | <60 seconds | 5-10 questions |

---

## Next Steps

1. **Try each skill**: Work through the examples above
2. **Combine skills**: Request multi-skill workflows for complete lesson development
3. **Customize**: Modify descriptions, add references, adjust for your teaching style
4. **Share feedback**: Report what works and what needs improvement

---

## Support & Resources

- **Spec**: `specs/001-colearning-skills/spec.md` - Full requirements
- **Research**: `specs/001-colearning-skills/research.md` - Design rationale
- **Data Models**: `specs/001-colearning-skills/data-model.md` - Entity structures
- **Contracts**: `specs/001-colearning-skills/contracts/` - YAML schemas

---

**Quick Reference: Trigger Phrases**

| Skill | Trigger Phrases |
|-------|----------------|
| learning-objectives | "define objectives", "learning goals", "prerequisites" |
| concept-scaffolding | "break down", "scaffold", "progressive steps" |
| code-example-generator | "generate example", "show code for", "demonstrate" |
| exercise-designer | "create exercises", "practice problems", "design activities" |
| technical-clarity | "review for clarity", "check readability", "improve explanation" |
| assessment-builder | "create quiz", "build assessment", "generate questions" |

---

**Ready to build effective educational content!** 🎓
