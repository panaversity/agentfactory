---
title: "Zed AI Features & Workflows"
chapter: 8
lesson_number: 3
proficiency_level: "A2-B1"
estimated_time: "75-90 minutes"
learning_objectives:
  - "Navigate Zed's inline assistant to generate code interactively"
  - "Configure multiple AI models in Zed for different tasks"
  - "Use tab autocomplete to accelerate writing with AI suggestions"
  - "Apply AI-powered refactoring to improve code structure"
  - "Generate git commit messages with AI"
  - "Work through a complete mini-project using Zed's AI features"
  - "Evaluate AI-generated code for correctness and quality"
skills:
  zed-ide:
    proficiency: "B1"
  ai-collaboration:
    proficiency: "B1"
  code-evaluation:
    proficiency: "A2-B1"
generated_by: content-implementer v1.0.0
source_spec: specs/029-chapter8-ai-native-ides/spec.md
created: "2025-11-20"
last_modified: "2025-11-20"
git_author: "Content Implementation"
workflow: /sp.implement
version: 1.0.0
---

# Zed AI Features & Workflows

You've installed Zed and connected it to an AI provider. Now you're ready to explore how Zed's features enable rapid, collaborative development with AI. In this lesson, you'll move from basic installation to practical workflows—learning how Zed integrates AI assistance into every coding activity, from code generation to refactoring to commit messages.

This lesson teaches you to observe and evaluate AI-generated code—developing critical judgment about when AI suggestions are excellent, adequate, or need refinement. By the end, you'll have completed a full mini-project using Zed's AI features, and you'll understand how to select the right AI feature for different development tasks.

## Opening: From Installation to Collaboration

Think back to Chapter 5 when you learned Claude Code. You opened the terminal, typed commands, and Claude executed them in a sandboxed environment. That's **asynchronous collaboration**—you request something, AI delivers results, you evaluate and iterate. There's a context switch: from your development environment to the terminal, back to your code.

Zed introduces **synchronous collaboration**. Your AI assistant is always visible in your editor, responding to prompts in milliseconds, suggesting improvements as you type, adapting to your project context. The assistant isn't separate from your work—it's embedded in your editing environment. You never leave your editor.

This changes how you develop:
- **Before (traditional)**: Write code manually → Test → Debug → Refactor (hours of work)
- **Now (AI-native)**: Describe intent to AI → Evaluate suggestions → Refine iteratively → Code emerges (minutes)

In this lesson, you'll experience this shift through Zed's core AI features. Each feature solves a specific development challenge. Master them, and your productivity multiplies.

---

## Section 1: The Inline Assistant — Your AI Coding Partner

The **inline assistant** is Zed's primary interface for AI collaboration. It's how you request code generation, ask questions about code structure, and iterate on suggestions—all without leaving your editor. Unlike copying code snippets from a web search or pasting from ChatGPT's web interface, the inline assistant is embedded in your editing experience—it understands your project structure, your existing code, and your development context in real-time.

### How the Inline Assistant Works

Open any file in Zed and press **Ctrl+I** (macOS: **Cmd+I**). A prompt box appears at your cursor location. Type your request in natural language, and Zed's AI generates code inline. The assistant runs in your editor process, giving it immediate access to your file and project context without any network delays after initial authentication.

**Basic workflow**:

1. **Position your cursor** where you want code inserted (inside a function, at end of file, between existing functions, etc.)
2. **Press Ctrl+I** to open the inline assistant
3. **Describe what you want** in natural language: "Write a function that converts Celsius to Fahrenheit"
4. **Press Enter** or **Tab** to accept the code, or **Escape** to cancel
5. **Iterate** — If the result isn't perfect, press Ctrl+I again and refine: "Add error handling for non-numeric input"

The beauty of this workflow is **incremental refinement**. You don't need to specify everything upfront. Start with a basic request, get code, then iteratively enhance it. This mirrors real-world development—build the core, then add robustness.

### Example: Simple Function Generation

Let's walk through a real example. You're starting a temperature converter project. You open a new file `converter.py` and position your cursor at the top.

**Your prompt** (Ctrl+I):
```
Write a function that converts Celsius to Fahrenheit.
Include the formula as a comment.
```

**What Zed generates**:
```python
def celsius_to_fahrenheit(celsius):
    """Convert temperature from Celsius to Fahrenheit.

    Formula: F = (C × 9/5) + 32
    """
    fahrenheit = (celsius * 9/5) + 32
    return fahrenheit
```

**What you observe**:
- The function is syntactically correct Python
- The docstring explains the conversion clearly
- The formula comment is included as requested
- The variable names are clear and descriptive
- No imports are needed—the code is self-contained
- The implementation is mathematically correct

You now have working code without typing a single character. The AI understood your intent and executed it correctly. More importantly, you can run this immediately: no syntax errors, no import issues, no cryptic variable names to decipher.

### Iterating on AI Suggestions

The inline assistant isn't one-shot. You can refine suggestions through iteration. This is where the real power of AI-native development emerges—rather than struggling with edge cases individually, you collaborate with AI to discover and handle them.

After the initial function, you want validation. Position your cursor at the end of the function and press Ctrl+I:

**Your refined prompt**:
```
Add error handling. If input is not a number, raise a TypeError with a clear message.
Also handle the edge case where input is a boolean (in Python, bool is a subclass of int).
```

**What Zed generates**:
```python
def celsius_to_fahrenheit(celsius):
    """Convert temperature from Celsius to Fahrenheit.

    Formula: F = (C × 9/5) + 32

    Args:
        celsius: Temperature value in Celsius (numeric, non-boolean)

    Returns:
        Temperature value in Fahrenheit

    Raises:
        TypeError: If input is boolean or non-numeric
    """
    if isinstance(celsius, bool):
        raise TypeError("Boolean input is not allowed")
    if not isinstance(celsius, (int, float)):
        raise TypeError(f"Expected numeric input, got {type(celsius).__name__}")
    fahrenheit = (celsius * 9/5) + 32
    return fahrenheit
```

**What emerged from this iteration**:
- The AI added defensive type checking using isinstance()
- It handled the boolean edge case FIRST (important because bool is subclass of int in Python)
- The error message is specific and helpful
- The docstring expanded to include Args, Returns, and Raises sections
- The logic is now production-grade with explicit validation

Through dialogue, your code became more robust than what either you or AI alone would produce. The AI learned your concern about boolean inputs and structured the validation correctly. You learned that Python has type hierarchy quirks worth handling explicitly.

### Understanding Context in the Inline Assistant

The inline assistant doesn't work in isolation. It reads your entire file context intelligently, understanding:
- **Existing functions** — What functions already exist, their signatures, their purposes
- **Imports and dependencies** — What libraries you've imported, preventing duplicate imports
- **Your coding style** — Variable naming conventions, comment style, documentation patterns
- **Related functions** — Functions in nearby files that this code should integrate with
- **Constants and configuration** — Named constants you've defined, configuration patterns

This contextual awareness prevents conflicts and enables intelligent suggestions. For example, if your file already has a `celsius_to_fahrenheit` function, the AI won't generate a duplicate. It will either suggest alternatives or ask for clarification.

**Pro tip**: The larger your project context, the smarter the AI becomes. This is why Zed's project-wide context awareness matters—the AI understands your entire codebase, not just the current file. This is fundamentally different from asking ChatGPT on the web, where the AI sees only what you paste.

### Advanced Iteration Patterns

Beyond simple "add feature" requests, try these patterns:

**Pattern 1: Gradual Enhancement**
- Request 1: "Write function X"
- Request 2: "Add error handling"
- Request 3: "Add type hints"
- Request 4: "Add detailed docstring"

Each iteration builds on the previous, keeping complexity manageable.

**Pattern 2: Alternative Implementations**
- Request 1: "Write function X using approach A"
- Request 2: "Rewrite using approach B instead"
- Request 3: "Which approach is better? Explain trade-offs"

This teaches you design alternatives by seeing them side-by-side.

**Pattern 3: Debugging Collaboration**
- Request 1: "This code doesn't work. Why?"
- Request 2: "Fix the issue"
- Request 3: "Why did that fix it?"

The AI explains problems AND solutions, teaching you in real-time.

---

## Section 2: Multi-Model Configuration — Choosing the Right AI for the Task

Different AI models excel at different tasks. Claude Sonnet is excellent at complex reasoning and thorough, production-ready generation. GPT-4 Mini is faster for quick completions, ideal for autocomplete and simple suggestions. Gemini 2.0 Flash excels at structured data tasks and documentation. Zed lets you configure multiple models and switch between them per task, enabling you to get the best tool for each specific need without compromising on either speed or quality.

The multi-model paradigm represents a fundamental shift in how you approach development. Before AI-native IDEs, you chose one editor, one AI provider, one set of capabilities. Now you can mix and match: use the most capable model for complex tasks, use the fastest model for quick suggestions, use the most specialized model for specific domains. This is **contextual optimization**—adapting your tools to the task at hand.

### Understanding Model Trade-offs

Before diving into configuration, understand what trade-offs each model makes:

**Claude Sonnet (by Anthropic)**:
- **Strengths**: Excellent reasoning, handles complex multi-step logic, produces well-documented code, understands edge cases, explains its thinking
- **Weaknesses**: Slightly slower than alternatives, uses more tokens for comprehensive solutions
- **Best for**: Complex algorithms, refactoring, code review, teaching examples, security-conscious coding
- **Cost**: ~$0.03 per 1M input tokens

**GPT-4 Mini (by OpenAI)**:
- **Strengths**: Fast responses, good at pattern recognition, lightweight code suggestions
- **Weaknesses**: Less nuanced reasoning, sometimes misses edge cases
- **Best for**: Autocomplete, quick suggestions, simple functions, boilerplate
- **Cost**: ~$0.00015 per 1M input tokens (very cheap)

**Gemini 2.0 Flash (by Google)**:
- **Strengths**: Fast, good at structured data, excellent at summarization and explanation
- **Weaknesses**: Different reasoning style
- **Best for**: Documentation, comments, data processing
- **Cost**: ~$0.075 per 1M input tokens

### Setting Up Multiple Models

Open Zed's settings (`Cmd+,` on macOS, `Ctrl+,` on Windows/Linux). You'll find your settings in a JSON file. Locate or create the AI configuration section:

**Basic configuration**:
```json
{
  "assistant": {
    "default_model": "claude-3-5-sonnet-20241022",
    "button": true
  },
  "inline_assistant": {
    "button": true
  },
  "tab_completion": {
    "disable": false,
    "debounce_duration_ms": 500,
    "max_output_tokens": 1024,
    "model": "gpt-4o-mini"
  }
}
```

This configuration strategically assigns:
- **claude-3-5-sonnet** for inline assistant (complex, user-initiated tasks)
- **gpt-4o-mini** for tab completion (simple, continuous suggestions)

**Why this split works**: Inline assistant is triggered for specific, often complex tasks. You're willing to wait 300-500ms for excellent results. Tab completion runs continuously as you type. If it were slow, it would distract you. Fast, good-enough suggestions beat slow, perfect suggestions for autocomplete.

### When to Use Each Model

Think about these scenarios:

| Task | Model | Reasoning |
|------|-------|-----------|
| **Complex algorithm** | Claude Sonnet | Needs deep reasoning |
| **Quick suggestions** | GPT-4o Mini | Speed matters more |
| **Refactor code** | Claude Sonnet | Needs to understand intent |
| **Generate boilerplate** | GPT-4o Mini | Simple pattern matching |
| **Documentation** | Gemini Flash | Summarization strength |
| **Security code** | Claude Sonnet | Careful reasoning needed |
| **Auto-complete** | GPT-4o Mini | Fast pattern recognition |
| **Explain legacy code** | Claude Sonnet | Complex logic understanding |

### Practical Example: Model Selection in Action

You're working on a temperature converter. You need to add several features. Watch how model choice impacts the workflow.

**Task 1: Generate formatting function** — Use Claude Sonnet (complex logic needed)

Inline assistant (Ctrl+I):
```
Write a function format_temperature(celsius, fahrenheit) that returns a human-readable string.
Handle edge cases where either value is None.
Include proper rounding to 1 decimal place.
The output should be formatted like "25.5°C (77.9°F)" when both values exist.
```

**Claude response** (comprehensive, handles all requirements):
```python
def format_temperature(celsius: float | None, fahrenheit: float | None, decimals: int = 1) -> str:
    """Format temperature values as human-readable strings.

    Args:
        celsius: Temperature in Celsius or None
        fahrenheit: Temperature in Fahrenheit or None
        decimals: Number of decimal places (default 1)

    Returns:
        Formatted string. Examples:
            - "25.5°C (77.9°F)" if both values provided
            - "25.5°C" if only Celsius provided
            - "No data" if neither provided
    """
    if celsius is None and fahrenheit is None:
        return "No data"

    parts = []
    if celsius is not None:
        parts.append(f"{celsius:.{decimals}f}°C")
    if fahrenheit is not None:
        parts.append(f"{fahrenheit:.{decimals}f}°F")

    return " (".join(parts) + ")" if len(parts) == 2 else parts[0]
```

Notice Claude's response: comprehensive docstring with examples, flexible parameters, clear error cases. This is the "everything you need" response.

**Task 2: Add quick comment** — Use GPT-4o Mini (simple, quick)

Inline assistant with Mini (Ctrl+I), after the function:
```
Add a one-line comment explaining what this function does.
```

**GPT-4o Mini response** (quick, adequate):
```python
# Formats temperature values with symbols, handling missing values
```

Done in 50ms. Claude would produce something similar but take 300ms. For this simple task, speed wins.

---

## Section 3: Tab Autocomplete — Real-Time Code Suggestions

While the inline assistant requires explicit prompts, **tab autocomplete** runs continuously as you type. Zed's AI watches your code and suggests the next logical lines—without you asking.

### How Tab Autocomplete Works

Tab autocomplete is always active (unless disabled). As you type, Zed analyzes your code and predicts what comes next. Ghost text appears in light gray, showing the suggestion. Press **Tab** to accept it, or keep typing to ignore it.

**Example in action**:

You type:
```python
def celsius_to_fahrenheit(
```

Ghost text appears:
```python
def celsius_to_fahrenheit(
    [celsius: float) -> float:]
```

Press **Tab** and it accepts. Your code is now:
```python
def celsius_to_fahrenheit(celsius: float) -> float:
```

The AI inferred you wanted type hints and suggested a complete signature.

### Configuring Autocomplete

Tab autocomplete settings live in settings.json:

```json
"tab_completion": {
  "disable": false,
  "debounce_duration_ms": 500,
  "max_output_tokens": 1024,
  "model": "gpt-4o-mini"
}
```

- **disable**: Set to `true` to turn off autocomplete
- **debounce_duration_ms**: Delay before suggesting (500ms = wait half-second after you stop typing)
- **max_output_tokens**: Max length of suggestions
- **model**: Which AI to use (GPT-4 Mini recommended for speed)

### When Autocomplete Accelerates You

Autocomplete shines for **predictable code patterns**. The AI learns what you're likely to type next based on context.

**High-leverage scenarios**:

1. **Function signatures** — Type `def ` and the AI suggests the full signature
2. **Common patterns** — Type `if __name__` and it completes to `if __name__ == "__main__":`
3. **List comprehensions** — Type `[x for` and it suggests the full comprehension
4. **Import statements** — Type `from typing` and it suggests common imports

**Low-leverage scenarios**:

1. **Novel logic** — When writing something unique
2. **Complex algorithms** — Custom, domain-specific logic
3. **One-off scripts** — Code that won't repeat

**Pro tip**: Autocomplete is most valuable when writing boilerplate and common patterns. Let it handle the routine stuff while you focus on the unique logic.

---

## Section 4: AI-Powered Code Refactoring

You've written working code. Now you want to improve it. **Refactoring with AI** lets you select code and ask the AI to improve it while preserving functionality.

### The Refactoring Workflow

1. **Select code** you want to improve (highlight it)
2. **Open the inline assistant** (Ctrl+I)
3. **Describe the improvement** — "Extract this into a separate function" or "Make this more Pythonic"
4. **Accept or iterate** on the result

### Example: Extracting Validation Logic

You've written a validation function inline, but it's getting messy:

```python
def process_temperature(value, unit):
    # Validation mixed with processing
    if not isinstance(value, (int, float)):
        raise TypeError(f"Expected number, got {type(value).__name__}")
    if isinstance(value, bool):
        raise TypeError("Cannot process boolean")
    if value < -273.15 and unit == 'C':
        raise ValueError("Temperature below absolute zero")

    # Processing
    if unit == 'C':
        return celsius_to_fahrenheit(value)
    elif unit == 'F':
        return fahrenheit_to_celsius(value)
    else:
        raise ValueError(f"Unknown unit: {unit}")
```

You select the validation section and press Ctrl+I:

**Your prompt**:
```
Extract the validation logic into a separate function called validate_temperature.
Keep the original function calling it.
```

**What Zed generates**:

First, it extracts the validation:
```python
def validate_temperature(value, unit):
    """Validate temperature value and unit."""
    if not isinstance(value, (int, float)):
        raise TypeError(f"Expected number, got {type(value).__name__}")
    if isinstance(value, bool):
        raise TypeError("Cannot process boolean")
    if value < -273.15 and unit == 'C':
        raise ValueError("Temperature below absolute zero")
```

Then it simplifies the original function:
```python
def process_temperature(value, unit):
    validate_temperature(value, unit)

    if unit == 'C':
        return celsius_to_fahrenheit(value)
    elif unit == 'F':
        return fahrenheit_to_celsius(value)
    else:
        raise ValueError(f"Unknown unit: {unit}")
```

**What emerged**: The code is now modular. Validation is testable in isolation. The main function reads more clearly.

---

## Section 5: Git Integration — AI-Generated Commit Messages

You've made changes to your code. Now you need to commit them. Writing good commit messages takes thought—describing what changed and why. Zed can generate these for you.

In your git diff, Zed's AI can analyze what changed and suggest an appropriate commit message. The AI understands:
- What lines were added/removed/modified
- The purpose of the change (feature, fix, refactor, docs)
- Specific file impacts

**Example commit from our temperature converter work**:

Your git diff shows:
```diff
+ def validate_temperature(value, unit):
+     """Validate temperature value and unit."""
+     if not isinstance(value, (int, float)):
+         raise TypeError(...)
+     if isinstance(value, bool):
+         raise TypeError(...)
+     if value < -273.15 and unit == 'C':
+         raise ValueError(...)

  def process_temperature(value, unit):
-     if not isinstance(value, (int, float)):
-         raise TypeError(...)
-     if isinstance(value, bool):
-         raise TypeError(...)
-     if value < -273.15 and unit == 'C':
-         raise ValueError(...)
+     validate_temperature(value, unit)
```

**AI-generated commit message**:
```
refactor: extract temperature validation into separate function

- Move validation logic from process_temperature to validate_temperature
- Improves modularity and makes validation independently testable
- Reduces process_temperature from 15 to 3 lines
```

The AI understood the change type (`refactor`), what was extracted and why, and the benefits. A good commit message! Better than "update temperature code."

---

## Section 6: Mini-Project: Temperature Converter with Zed

Now you'll put it all together. You'll build a complete temperature converter CLI using Zed's AI features.

### Project Scope

**Goal**: Build a Python CLI that converts between Celsius and Fahrenheit

**Features**:
1. Convert single temperature (C→F or F→C)
2. Convert temperature ranges
3. Validate input and provide helpful error messages
4. Display results in formatted output

**Success criteria**:
- Program accepts command-line arguments
- Output is human-readable
- All user inputs are validated
- Program handles errors gracefully

### Step 1: Project Setup

Create a new directory for your project:

```bash
mkdir temp_converter
cd temp_converter
git init
```

In Zed, open this folder. Create a new file `converter.py`.

### Step 2: Core Conversion Functions

In your `converter.py`, position cursor at the top. Press Ctrl+I:

**Your prompt**:
```
Write two functions: celsius_to_fahrenheit and fahrenheit_to_celsius.
Include type hints and docstrings.
Add validation to reject boolean inputs.
Include the conversion formulas as comments.
```

**Expected output**:
```python
def celsius_to_fahrenheit(celsius: float) -> float:
    """Convert temperature from Celsius to Fahrenheit.

    Formula: F = (C × 9/5) + 32

    Args:
        celsius: Temperature in Celsius (must be numeric, not boolean)

    Returns:
        Temperature in Fahrenheit

    Raises:
        TypeError: If input is boolean or non-numeric
        ValueError: If temperature is below absolute zero
    """
    if isinstance(celsius, bool):
        raise TypeError("Boolean input is not allowed")
    if not isinstance(celsius, (int, float)):
        raise TypeError(f"Expected float, got {type(celsius).__name__}")
    if celsius < -273.15:
        raise ValueError("Temperature cannot be below absolute zero (-273.15°C)")

    return (celsius * 9/5) + 32


def fahrenheit_to_celsius(fahrenheit: float) -> float:
    """Convert temperature from Fahrenheit to Celsius.

    Formula: C = (F - 32) × 5/9

    Args:
        fahrenheit: Temperature in Fahrenheit

    Returns:
        Temperature in Celsius

    Raises:
        TypeError: If input is boolean or non-numeric
        ValueError: If temperature is below absolute zero Fahrenheit
    """
    if isinstance(fahrenheit, bool):
        raise TypeError("Boolean input is not allowed")
    if not isinstance(fahrenheit, (int, float)):
        raise TypeError(f"Expected float, got {type(fahrenheit).__name__}")
    if fahrenheit < -459.67:
        raise ValueError("Temperature cannot be below absolute zero (-459.67°F)")

    return (fahrenheit - 32) * 5/9
```

**What you've accomplished**:
- Two working conversion functions
- Type hints for clarity
- Comprehensive docstrings
- Input validation (catches booleans explicitly)
- Boundary validation (absolute zero in both scales)

### Step 3: Formatting and Main CLI

Position cursor after the two conversion functions. Press Ctrl+I:

**Your prompt**:
```
Add a format_temperature function that takes celsius and fahrenheit values
and returns a formatted string like "25.0°C (77.0°F)".
Handle cases where one value might be None.

Also add a main() function that:
1. Takes command-line arguments using sys.argv
2. Accepts format: python converter.py <value> <unit>
3. Converts to the other unit
4. Prints formatted output
5. Handles errors gracefully

Include the import statement for sys at the top.
```

**Expected output**:
```python
import sys


def format_temperature(celsius: float | None, fahrenheit: float | None) -> str:
    """Format temperature values as a readable string."""
    if celsius is None and fahrenheit is None:
        return "No temperature data"

    parts = []
    if celsius is not None:
        parts.append(f"{celsius:.1f}°C")
    if fahrenheit is not None:
        parts.append(f"{fahrenheit:.1f}°F")

    return " / ".join(parts)


def main():
    """Command-line interface for temperature conversion."""
    if len(sys.argv) != 3:
        print("Usage: python converter.py <temperature> <unit>")
        print("  temperature: numeric value")
        print("  unit: 'C' (Celsius) or 'F' (Fahrenheit)")
        sys.exit(1)

    try:
        temp_value = float(sys.argv[1])
        unit = sys.argv[2].upper()

        if unit == 'C':
            result_f = celsius_to_fahrenheit(temp_value)
            output = format_temperature(temp_value, result_f)
            print(f"Input: {temp_value}°C")
            print(f"Output: {output}")
        elif unit == 'F':
            result_c = fahrenheit_to_celsius(temp_value)
            output = format_temperature(result_c, temp_value)
            print(f"Input: {temp_value}°F")
            print(f"Output: {output}")
        else:
            print(f"Error: Unknown unit '{unit}'. Use 'C' or 'F'.")
            sys.exit(1)

    except ValueError:
        print(f"Error: '{sys.argv[1]}' is not a valid number.")
        sys.exit(1)
    except TypeError as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
```

### Step 4: Testing Your Code

Test what you've built. In Zed's terminal:

```bash
python converter.py 0 C
```

Expected output:
```
Input: 0°C
Output: 0.0°C / 32.0°F
```

Try more tests:
```bash
python converter.py 32 F    # Should show 0.0°C
python converter.py 100 C   # Should show 212.0°F
python converter.py "abc" C # Should show error message
```

### Step 5: Commit Your Work

In your terminal:

```bash
git add converter.py
git commit -m "feat: implement temperature converter CLI with validation

- Add celsius_to_fahrenheit and fahrenheit_to_celsius functions
- Include temperature validation and boundary checks
- Create CLI interface accepting command-line arguments
- Add formatted output with symbol display"
```

---

## Section 7: Evaluating AI-Generated Code — Critical Thinking

You've used Zed's AI extensively. Now develop critical evaluation skills. Not all AI code is production-ready. Sometimes you need to refine, reject, or improve suggestions.

### The Evaluation Framework

When AI generates code, ask these questions:

**Correctness**:
- Does the code do what you asked?
- Does it handle edge cases?
- Would it work on actual data?

**Quality**:
- Are variable names clear?
- Is the code readable?
- Does it follow Python conventions?

**Performance**:
- Is it efficient?
- Could it be faster?

**Maintainability**:
- Is it easy to modify later?
- Are there helpful comments?

### Red Flags in AI Code

Learn to spot problems quickly:

| Red Flag | What It Means | Example |
|----------|---------------|---------|
| No type hints | Unclear contract | `def convert(temp)` |
| No docstring | Undocumented | Missing explanation |
| No error handling | Assumes perfect input | Crashes on invalid data |
| Hardcoded values | Not reusable | `temp * 1.8 + 32` |
| Repetitive code | Not DRY | Same logic repeated |
| Unclear names | Hard to maintain | `x = y * 9/5 + 32` |

When you spot red flags, use the inline assistant to improve: "Add type hints", "Extract this into a constant", "Add error handling".

---

## Section 8: Common Workflows and Time-Saving Patterns

You've learned individual Zed AI features. Now combine them into complete workflows.

### Workflow 1: From Concept to Working Code (10 minutes)

**Scenario**: You have an idea. You want working code quickly.

1. **Open Zed** with your project
2. **Position cursor** in new file
3. **Ctrl+I** — Large prompt describing entire feature
4. **Accept the suggestion** — AI generates substantial code
5. **Tab autocomplete** — Fill in remaining pieces
6. **Quick refactor** — Extract any messy sections
7. **Test** — Run the code, verify it works

**Time savings**: What takes 45 minutes manually takes 10 with AI.

### Workflow 2: Improve Existing Code (5 minutes)

**Scenario**: You have working code that's messy. You want it production-ready.

1. **Select code block** you want to improve
2. **Ctrl+I** — "Improve this code for clarity/performance"
3. **Review suggestion**
4. **Accept or iterate** until satisfied

**Time savings**: Refactoring takes 5 minutes instead of 20.

### Workflow 3: Code Review (10 minutes)

**Scenario**: AI generated code, but you want a second opinion.

1. **Ask AI a question** about the code: "Are there edge cases this doesn't handle?"
2. **Review response**
3. **Iterate** — "Add handling for [specific case]"
4. **Test** — Verify improvements work

**Time savings**: Catching bugs before they reach production.

---

## Advanced Section: Understanding Zed's AI Architecture and Capabilities

To use Zed effectively, understanding its underlying architecture helps you work WITH the AI rather than against it. Zed is fundamentally different from traditional IDE plugins because of how it integrates AI at the architectural level.

### How Zed Sees Your Code

When you press Ctrl+I, Zed doesn't send just your cursor position to the AI. It sends:

1. **Full file context** — Everything in your current file (not just the function you're editing)
2. **Project structure** — The names and locations of nearby files, giving the AI awareness of your project organization
3. **File relationships** — Which files import from which, helping the AI understand dependencies
4. **Recent history** — Changes you've made recently in this session, so the AI learns your style
5. **Language context** — The programming language, which versions, which frameworks you're using
6. **Cursor position** — Exact insertion point, helping the AI understand where code should go

This is why Zed feels so much smarter than web-based tools. When you paste code into ChatGPT, ChatGPT sees only what you pasted. In Zed, the AI sees your entire project ecosystem. This enables suggestions that actually fit your codebase rather than generic patterns.

### The Performance/Quality Trade-off

Notice in your multi-model setup, you use fast models for autocomplete and slow models for inline assistant. This isn't arbitrary—it's a deliberate trade-off:

**Tab Autocomplete (GPT-4o Mini)**:
- User experience priority: Responsiveness
- Tolerance for mistakes: High (user can ignore suggestions easily)
- Context: Single line or few lines ahead
- Latency budget: Less than 200ms (faster than user can type)
- Quality acceptable: 70% (user accepts/rejects suggestions immediately)

**Inline Assistant (Claude Sonnet)**:
- User experience priority: Correctness
- Tolerance for mistakes: Low (generates larger code blocks)
- Context: Full file or multiple files
- Latency budget: Less than 2 seconds (user waits for generation)
- Quality acceptable: 95%+ (code should be production-grade)

By understanding these trade-offs, you can anticipate how Zed will behave and adjust your expectations accordingly.

### Language-Specific Considerations

Different programming languages benefit from different AI approaches. This is why Zed lets you configure per-language settings:

**Python** (what we're teaching):
- AI understands Python conventions well (PEP 8 style guide widely represented in training)
- Type hints are helpful (Claude understands Python's type system)
- Error messages are descriptive (makes debugging easier)
- Standard library is large and well-documented

**JavaScript/TypeScript**:
- Multiple style conventions (many developers don't use consistent formatting)
- Async/await patterns can confuse some AI models
- Dependency management (npm/yarn/pnpm) varies widely
- Framework ecosystem is vast (React, Vue, Angular, etc.)

**Go, Rust, C++** (systems languages):
- AI generally produces more verbose but safer code
- Memory management and ownership require careful reasoning
- Performance considerations are explicit
- Trade-offs between safety and flexibility are nuanced

When you switch languages, you might adjust model selection. For example, you might use Claude for all languages, but switch to GPT-4o for quick JavaScript suggestions.

### Error Recovery in AI-Assisted Development

Not every AI suggestion works perfectly. Understanding how to recover from mistakes is crucial.

**Scenario: AI Generates Code With a Bug**

```python
def celsius_to_fahrenheit(celsius):
    """Convert Celsius to Fahrenheit."""
    return celsius * 9/5 + 32  # Bug: forgot to handle the +32 offset
```

You run it: `converter(0)` returns 0, but should return 32. You catch the error immediately.

**Recovery approach**:

1. **Don't just fix it manually** — That defeats the purpose of AI assistance
2. **Ask the AI to diagnose** — Press Ctrl+I: "This function returns the wrong value. For input 0, it should return 32, but it returns 0. What's wrong?"
3. **AI explains** — "The formula is incomplete. The offset of +32 was never added"
4. **AI fixes** — "Add + 32 to the return statement"
5. **You learn** — Understanding why it failed helps you write better prompts next time

This collaborative debugging is where AI-native development shines. Rather than fixing bugs silently, you make the AI explain its mistakes. This teaches you and improves future suggestions.

### Prompt Engineering for Better Results

You've already learned basic inline assistant usage. Advanced users craft specific prompts to guide the AI:

**Weak prompt**:
```
Write a function to convert temperatures.
```

**Strong prompt**:
```
Write a Python function celsius_to_fahrenheit(celsius: float) -> float that:
- Converts a single Celsius temperature to Fahrenheit
- Uses the formula F = (C × 9/5) + 32
- Includes comprehensive docstring with Args, Returns, and Raises sections
- Validates that input is numeric (int or float) and not boolean
- Raises TypeError with descriptive message for non-numeric input
- Raises ValueError for temperatures below absolute zero (-273.15°C)
- Does NOT use external libraries (only built-in Python)
```

Notice the difference? The strong prompt:
- Specifies the exact function signature
- Includes the formula (removes ambiguity)
- Lists documentation requirements
- Specifies validation behavior
- Sets constraints (no external libraries)

The AI responds to specificity. More detail → Better results.

### Debugging With AI When Something's Not Working

Sometimes your code works syntactically but produces wrong results. Zed helps you debug:

**Scenario**: Your temperature converter gives wrong results

```python
def celsius_to_fahrenheit(celsius):
    return (celsius - 32) * 9/5  # Wrong formula!
```

Test: Input 0, expected 32, got -17.78. Obviously wrong.

**Debugging with AI**:

1. Select the function
2. Press Ctrl+I: "This function has a logic error. For input 0°C, it should return 32°F, but it returns -17.78. What's wrong with the formula?"

**AI response**:
```
The formula is inverted. You're using the Fahrenheit-to-Celsius formula instead of Celsius-to-Fahrenheit.

Correct formula: F = (C × 9/5) + 32
Your formula: F = (C - 32) × 9/5

The subtraction and order are reversed.
```

**AI fixes it**:
```python
def celsius_to_fahrenheit(celsius):
    return (celsius * 9/5) + 32  # Correct formula
```

This teaching moment is valuable. You learned the formula was backwards, why that's wrong, and how to verify it's correct.

### Context Limits and What To Do When They're Reached

Zed can only send so much context to the AI before it exceeds token limits. For Claude Sonnet, context limit is ~200K tokens (roughly 150,000 words). For your temperature converter, you won't hit this limit. But for large projects:

**If you hit context limits**:
1. Zed will notify you
2. The AI will see only the most relevant files
3. Suggestions might be less accurate for distant parts of your codebase

**Solutions**:
- Break large features into smaller, focused tasks
- Make inline assistant requests very specific about which files matter
- Use version control (git) to keep projects organized so Zed can see structure clearly

For most educational and small-to-medium projects, context isn't a constraint. But be aware it exists.

### Transitioning From Zed to Production Development

This lesson teaches you Zed fundamentals using a simple temperature converter. In real professional work, you'll encounter more complex scenarios:

- **Large codebases** with hundreds of files
- **Complex frameworks** (Django, FastAPI, etc.) with many interdependencies
- **Performance-critical code** where AI suggestions need benchmarking
- **Security-sensitive code** where AI output requires review
- **Team projects** where code style must match existing conventions

The fundamentals you learn here transfer directly. The inline assistant still works for large projects—it just requires more specific prompts. Tab autocomplete still saves you from boilerplate. The refactoring workflow still improves code quality.

But in production, add these practices:

1. **Always review AI-generated code** before shipping
2. **Write tests** to verify AI-generated code works correctly
3. **Use version control** to track AI-assisted changes
4. **Document decisions** about which model/approach you chose and why
5. **Keep learning** — AI models improve, stay current with new features

---

## Summary: Mastering Zed's AI Workflows

By completing this lesson, you've learned:

**Core Features**:
- Inline assistant (Ctrl+I) for explicit code generation requests
- Tab autocomplete for continuous, lightweight suggestions
- Multi-model configuration to match models to tasks
- Code refactoring through selection and prompting
- Git integration for AI-assisted commit messages

**Advanced Concepts**:
- How Zed provides project context to AI (unlike web tools)
- Performance/quality trade-offs in model selection
- How to write effective prompts for better results
- Error recovery and collaborative debugging
- When AI is helpful vs. when you should code manually

**Practical Skills**:
- Building a complete CLI application with AI assistance
- Evaluating AI-generated code critically
- Iterating on suggestions to improve quality
- Using common development workflows with AI

**Pedagogical Insight**:
- You've experienced **Layer 2 AI Collaboration** (AI as Teacher/Student/Co-Worker)
- You've evaluated AI output rather than passively accepting it
- You've learned that AI assistance requires critical thinking, not blind trust

In the next lesson, you'll install **Cursor IDE** and apply these same skills to a different tool. You'll notice similarities in concepts but differences in implementation. This comparative experience will deepen your understanding of what makes IDEs "AI-native" and help you choose the right tool for different contexts.

The temperature converter project you built is modest in scope but complete in functionality. In professional contexts, you'll build much larger systems. But the workflow remains the same: collaborate with AI to generate code, evaluate it critically, iterate to improve, and commit your work to version control.

You're now ready to become proficient in multiple AI-native IDEs.
