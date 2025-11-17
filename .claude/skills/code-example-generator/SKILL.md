---
name: code-example-generator
version: 2.0
description: |
  Generates runnable, pedagogically sound code examples (Python and TypeScript) with validation for teaching purposes.
  Activate when authors need teaching examples that demonstrate concepts clearly, request code snippets
  for educational content, or need examples validated for correctness and best practices. Creates
  progressive example sequences (simple → realistic → complex) with clear explanations of what, how,
  and why. Use when generating examples for: functions, data structures, OOP, control flow, error
  handling, or any concept requiring demonstration through working code.
version: "2.1.0"
constitution_alignment: v4.0.1
---

## Purpose

The code-example-generator skill helps authors create runnable, well-structured code examples (Python and TypeScript) that clearly demonstrate specific concepts for learners. This skill produces examples that are pedagogically sound, follow best practices, and are validated for correctness through syntax checking and optional sandbox execution.

**Constitution v4.0.1 Alignment**: This skill implements Principle 1 (Specification Primacy)—teaching students that specification clarity is the primary skill, not code writing.

### Requirements (Evals-First, Then Spec-First, Then Implementation)

**CRITICAL WORKFLOW** (Constitution v4.0.1, aligned with Section IIa for Layer 4 spec-driven work):
1. **Evals First**: Define success criteria (what makes this example "good"?) BEFORE writing spec
2. **Spec Second**: Reference approved chapter specification (path to spec file)
3. **Prompt Third**: Document exact AI prompt(s) used to generate code
4. **Code Fourth**: Generate the implementation
5. **Validation Fifth**: Validate against predefined evals (tests, scripts, criteria)

**MANDATORY Template for First Example in Lesson**:
```markdown
### Spec → Prompt → Code → Validation

**Spec**: `specs/part-X/chapter-Y/spec.md` (approved)
**Success Evals**: [List criteria defined in spec]
**Prompt**: "Generate a Python function that..."
**Generated Code**: [Code below]
**Validation**:
- Syntax: ✓ Pass (validate-syntax.py)
- Tests: ✓ Pass (all test cases)
- Evals: ✓ Meets success criteria
```

**Do NOT** generate examples without:
- ✅ Reference to approved spec
- ✅ Predefined success evals
- ✅ Exact AI prompt documented
- ✅ Validation plan and results

### Minimal test snippet template
```
def test_example_basic():
    # Arrange
    # [setup]
    # Act
    # [call]
    # Assert
    assert ...
```

### Security lint checklist
- [ ] No hardcoded secrets/tokens
- [ ] Input validation present when parsing external data
- [ ] Safe defaults and error handling

### Bilingual Development (Python + TypeScript)

**When to provide both languages** (Constitution v3.1.2):
- ✅ Fundamental concepts that apply to both (functions, classes, async/await)
- ✅ When audience includes full-stack developers
- ✅ When concept has language-specific nuances worth highlighting
- ❌ Python-specific features (decorators, context managers) → Python only
- ❌ TypeScript-specific features (interfaces, generics) → TypeScript only

**Format for bilingual examples**:
```markdown
### Python Implementation
[Python code with comments]

### TypeScript Implementation
[TypeScript code with comments]

### Key Differences
- [Language-specific considerations]
```

**Default**: Python examples unless spec explicitly requests TypeScript or bilingual coverage.

## When to Activate

Use this skill when:
- Authors need code examples for teaching specific Python concepts
- Generating demonstration code for tutorials, books, or courses
- Creating progressive example sequences (simple to complex)
- Validating existing code examples for pedagogical soundness
- Need examples that follow Python best practices (PEP 8)
- Examples must be runnable and error-free
- Demonstrating concepts like: functions, classes, data structures, comprehensions, error handling, file I/O, etc.

## Inputs

Required:
- **Concept to demonstrate**: The Python concept/feature to show (e.g., "list comprehensions")
- **Learning level**: beginner | intermediate | advanced

Optional:
- **Specific requirements**: What the example should accomplish
- **Constraints**: Things to avoid or include
- **Context**: Where example will be used (book, tutorial, lecture, etc.)
- **Progressive sequence**: Whether this is part of a series of examples

## Process

### Step 1: Understand the Concept and Audience

Clarify:
- What specific aspect of the concept to demonstrate
- What the learner should understand after seeing this example
- What prerequisites the learner has
- Common misconceptions or difficulties with this concept

### Step 2: Load Best Practices Reference

Read Python best practices for guidance on style and conventions:

```bash
Read reference/python-best-practices.md
```

Key considerations:
- PEP 8 style guidelines
- Type hints (when appropriate for level)
- Docstring conventions
- Naming standards
- Level-appropriate complexity

### Step 3: Load Example Patterns

Read effective teaching example patterns:

```bash
Read reference/example-patterns.md
```

Apply patterns:
- One concept per example
- Progressive complexity (simple → realistic → complex)
- Show expected output
- Make examples runnable and self-contained
- Include clear comments explaining reasoning

### Step 4: Design Example Structure

Choose appropriate pattern:
- **Simple demonstration**: Basic usage of concept
- **Before/After**: Show old way vs new way
- **Problem → Solution**: State problem, show solution
- **Incremental building**: Add complexity step by step
- **Common mistake → Fix**: Show error, then correction
- **Multiple approaches**: Compare different solutions

### Step 5: Write the Code Example

Create example following these principles:

**For All Levels**:
- One primary concept per example
- Complete and runnable (no missing imports or undefined variables)
- Show expected output in comments
- Use meaningful variable names (not foo/bar)
- Include explanation of what, how, and why

**Beginner Examples**:
```python
# Clear, explicit, verbose
numbers = [1, 2, 3, 4, 5]
total = 0

for number in numbers:
    total = total + number  # Add each number to total

average = total / len(numbers)
print(f"Average: {average}")  # Average: 3.0
```

**Intermediate Examples**:
```python
# More Pythonic, use comprehensions and idioms
def calculate_average(numbers: list[int]) -> float:
    """Calculate the average of a list of numbers."""
    if not numbers:
        raise ValueError("Cannot calculate average of empty list")
    return sum(numbers) / len(numbers)

# Using the function
result = calculate_average([1, 2, 3, 4, 5])
print(f"Average: {result}")  # Average: 3.0
```

**Advanced Examples**:
```python
# Production-ready with comprehensive error handling
from typing import List, Union
from statistics import mean, StatisticsError

def calculate_statistics(
    numbers: List[Union[int, float]]
) -> dict[str, float]:
    """
    Calculate statistical measures for a list of numbers.

    Args:
        numbers: List of numeric values

    Returns:
        Dictionary with 'mean', 'min', 'max' keys

    Raises:
        ValueError: If numbers list is empty
        TypeError: If list contains non-numeric values
    """
    if not numbers:
        raise ValueError("Cannot calculate statistics of empty list")

    try:
        return {
            'mean': mean(numbers),
            'min': min(numbers),
            'max': max(numbers)
        }
    except StatisticsError as e:
        raise ValueError(f"Invalid data for statistics: {e}")

# Example usage with error handling
try:
    stats = calculate_statistics([1, 2, 3, 4, 5])
    print(f"Mean: {stats['mean']}")  # Mean: 3.0
except ValueError as e:
    print(f"Error: {e}")
```

### Step 6: Validate Syntax

Save the example code to a temporary file and validate syntax:

```bash
python .claude/skills/code-example-generator/scripts/validate-syntax.py example.py
```

The script will:
- Parse code into AST (Abstract Syntax Tree)
- Report any syntax errors with line numbers
- Check for common issues (missing docstrings, very long lines, etc.)
- Provide statistics (number of functions, classes, lines)

Review validation output and fix any syntax errors.

### Step 7: Optional - Execute in Sandbox

For complete validation, run code in isolated sandbox:

```bash
python .claude/skills/_shared/sandbox-executor.py example.py --timeout 5
```

The sandbox will:
- Execute code in isolated environment
- Capture stdout and stderr
- Enforce 5-second timeout
- Report execution success or errors

If execution fails:
1. Review error messages
2. Fix issues (logic errors, undefined variables, etc.)
3. Re-validate and re-execute
4. Iterate until code runs successfully

### Step 8: Add Pedagogical Annotations

Enhance example with:

**Clear Comments**:
```python
# Step 1: Initialize accumulator
total = 0

# Step 2: Sum all numbers
for num in numbers:
    total += num  # Accumulate sum

# Step 3: Calculate average
average = total / len(numbers)
```

**Expected Output**:
```python
print(result)  # Output: [1, 4, 9, 16, 25]
```

**Explanation Section**:
```markdown
## How It Works

1. **Initialization**: Create empty list for results
2. **Iteration**: Loop through each item
3. **Transformation**: Apply operation to each item
4. **Result**: New list with transformed values
```

### Step 9: Format Complete Example

Structure the final example following the template:

```bash
Read templates/code-example-template.md
```

Include:
- **What** this demonstrates
- **The code** (with comments)
- **How it works** (step-by-step explanation)
- **Why it matters** (practical value)
- **Common mistakes** to avoid
- **Variations** (alternative approaches)

## Output Format

Provide complete code example as structured markdown (include Spec/Prompt/Validation header on first example per lesson):

```markdown
# Example: [Title]

Spec/Prompt/Validation
```
Spec: [path/to/specs/part-X/chapter-Y-spec.md]
Prompt(s):
- "[Exact prompt text used]"
Validation:
- Syntax: scripts/validate-syntax.py (pass)
- Execution: _shared/sandbox-executor.py (pass)
- Tests: include minimal test snippet (passes)
```

### Failure mode example (intermediate+)
```
## Failure Case
Input: [bad input]
Expected: Raises ValueError with message "..."
```
```

**Concept**: [Primary concept]
**Level**: [beginner/intermediate/advanced]
**Prerequisites**: [List]

## The Code

```python
# [Complete, runnable code with comments]
```

## How It Works

[Step-by-step explanation]

## Why It Matters

[Practical application or value]

## Common Mistakes

**Mistake**: [Description]
```python
# Wrong
[incorrect code]
```

**Fix**: [How to correct]
```python
# Correct
[fixed code]
```

## Variations

[Alternative approaches if applicable]
```

## Examples

### Example 1: Generate Beginner Example for List Append

**Input**: "Generate a beginner example demonstrating list.append()"

**Process**:
1. Identify concept: Adding items to a list dynamically
2. Choose simple, relatable context (shopping list)
3. Write clear, verbose code with step-by-step comments
4. Show expected output
5. Validate syntax
6. Execute in sandbox
7. Add pedagogical annotations

**Output**:
```python
# Demonstrates: list.append() method
# Level: Beginner

# Create an empty shopping list
shopping_list = []

# Add items one at a time using append()
shopping_list.append("apples")
shopping_list.append("bananas")
shopping_list.append("milk")

# Display the complete list
print(shopping_list)
# Output: ['apples', 'bananas', 'milk']

# How it works:
# - append() adds an item to the END of the list
# - Each append() call adds one item
# - List grows dynamically as items are added
```

---

### Example 2: Generate Intermediate Example for Exception Handling

**Input**: "Show how to safely read user input and handle invalid numbers"

**Process**:
1. Concept: try/except with ValueError
2. Use realistic scenario (user input)
3. Show both error case and success case
4. Include type hints and docstring
5. Validate and execute

**Output**:
```python
# Demonstrates: Exception handling with try/except
# Level: Intermediate

def get_number_from_user() -> int | None:
    """
    Prompt user for a number and handle invalid input.

    Returns:
        Integer if valid input, None if invalid
    """
    try:
        user_input = input("Enter a number: ")
        number = int(user_input)  # Might raise ValueError
        return number
    except ValueError:
        print("Invalid input! Please enter a valid number.")
        return None

# Example usage
result = get_number_from_user()
if result is not None:
    print(f"You entered: {result}")
else:
    print("No valid number provided.")

# Why this works:
# - try block contains code that might fail
# - except ValueError catches conversion errors specifically
# - Returns None to indicate failure (explicit is better than implicit)
```

---

### Example 3: Review Existing Example for Issues

**Input**: "Review this code for pedagogical issues"
```python
def func(x,y):
    return x+y
print(func(1,2))
```

**Process**:
1. Validate syntax (passes)
2. Check best practices violations:
   - Missing docstring
   - Non-descriptive function name
   - No type hints
   - No spaces around operators (in function signature)
   - No explanation or comments
   - No context about what it demonstrates
3. Provide specific improvements

**Output**: Improved version:
```python
# Demonstrates: Function definition with parameters
# Level: Beginner

def add_numbers(first_number, second_number):
    """
    Add two numbers together.

    Args:
        first_number: The first number to add
        second_number: The second number to add

    Returns:
        The sum of the two numbers
    """
    result = first_number + second_number
    return result

# Example usage
sum_result = add_numbers(1, 2)
print(f"1 + 2 = {sum_result}")  # Output: 1 + 2 = 3
```

## Common Patterns

### Pattern 1: Progressive Sequence

Create 3 examples showing evolution:

**Stage 1**: Simplest possible demonstration
**Stage 2**: Add realistic features (error handling, parameters)
**Stage 3**: Production-ready with full validation

### Pattern 2: Before/After

Show traditional approach, then introduce Python idiom:

```python
# Before: Manual iteration
result = []
for item in items:
    if condition(item):
        result.append(item)

# After: List comprehension
result = [item for item in items if condition(item)]
```

### Pattern 3: Common Mistake → Fix

Demonstrate error, explain problem, show solution

## Validation Checklist

Before finalizing example:
- [ ] Code is syntactically correct (validated via script)
- [ ] Code executes without errors (tested in sandbox)
- [ ] Demonstrates ONE primary concept clearly
- [ ] Includes meaningful comments explaining reasoning
- [ ] Shows expected output
- [ ] Uses appropriate naming (no foo/bar)
- [ ] Follows PEP 8 style guidelines
- [ ] Appropriate complexity for target level
- [ ] Complete and runnable (no missing imports/variables)
- [ ] Includes docstrings for functions/classes

## References

Supporting documentation (loaded as needed):
- `reference/python-best-practices.md` - PEP 8, type hints, docstrings, naming
- `reference/example-patterns.md` - Effective teaching example structures
- `reference/pep8-summary.md` - Quick PEP 8 reference

## Error Handling

If validation fails:
1. Report specific syntax errors with line numbers
2. Identify which best practices are violated
3. Suggest specific corrections
4. If sandbox execution fails, report runtime errors
5. Halt and require user intervention (hard failure mode)

Do not proceed with invalid or failing examples—examples must be correct and runnable.
