---
title: "Technical Constraints and Examples — Make AI Code Fit Your Project"
chapter: 9
lesson: 6
duration_minutes: 45

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Specifying Technical Constraints"
    proficiency_level: "A2→B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student adds 3+ constraints that AI meets in generated code"

  - name: "Providing Style and Pattern Examples"
    proficiency_level: "A2→B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student includes example snippet; AI follows project style and error handling patterns"

learning_objectives:
  - objective: "Constrain AI outputs by versions, security, performance, and integration requirements"
    proficiency_level: "A2→B1"
    bloom_level: "Apply"
    assessment_method: "Exercise adding constraints to a basic prompt; verify outputs satisfy them"

  - objective: "Embed examples (style, API responses, tests) to guide AI toward project-specific code"
    proficiency_level: "A2→B1"
    bloom_level: "Apply"
    assessment_method: "Provide snippet; verify AI matches structure and conventions"

cognitive_load:
  new_concepts: 3
  assessment: "3 new concepts (technical constraints, code examples, integration constraints) fit A2→B1 transition ✓"

differentiation:
  extension_for_advanced: "Combine multiple constraint categories and compare tradeoffs; enforce via CI linters/tests"
  remedial_for_struggling: "Start with version + security only; supply a ready-made style snippet"
---

## What: Technical Constraints in AIDD

Constraints tell your AI collaborator exactly what the code must obey to work in your environment. Categories you’ll use often:

- Dependency constraints (language/framework versions, allowed libraries)
- Performance constraints (latency, memory, throughput)
- Security constraints (validation, rate limiting, crypto, access control)
- Code quality constraints (type hints, coverage, style)
- Integration constraints (existing services, logging, migrations, CI/CD)

These guardrails produce code that fits your project the first time—no costly rewrites later.

## Why: Constraints Ensure Project Fit

Without constraints, AI may choose the newest API, an unsupported version, or a pattern that doesn’t match your stack. With constraints, AI delivers code that compiles, deploys, and integrates on day one. Constraints are not bureaucracy—they’re how you convert “good code” into “our code.”

## How: Specifying Constraints and Using Examples

Start with explicit dependency versions and must/never rules, then add examples to shape style and patterns.

### Dependency Constraints (example)
```
Technical Constraints:
- Python version: 3.11 (use match-case; not Python 3.9 compatible)
- FastAPI version: 0.104.0 (use latest Annotated syntax)
- Database: PostgreSQL 15 with SQLAlchemy 2.0 (new ORM syntax)
- No additional dependencies beyond requirements.txt
- Must run in Docker (Alpine Linux base)
```

### Security and Code Quality Constraints (example)
```
Security Requirements:
- Validate and sanitize all inputs
- Use parameterized queries (no raw SQL)
- Rate limit: 100 requests/min/IP
- Passwords: bcrypt (cost factor 12)
- JWT: 1h access tokens with refresh mechanism
- No sensitive data in logs

Code Quality Standards:
- Type hints required for all functions
- Test coverage ≥ 80% (pytest-cov)
- Follow PEP 8 and Google docstrings
- Cyclomatic complexity < 10 per function
```

### Style Example (guide AI with patterns)
```
Match this error handling pattern:

try:
    result = await some_operation()
except DatabaseError as e:
    logger.error(f"Database error in operation: {str(e)}")
    raise HTTPException(status_code=500, detail="Database error occurred")
except ValidationError as e:
    logger.warning(f"Validation error: {str(e)}")
    raise HTTPException(status_code=400, detail=str(e))
```

### API Response Example (shape outputs)
```
Success Response:
{
  "status": "success",
  "message": "User created successfully"
}

Error Response:
{
  "status": "error",
  "error": {"code": "BAD_REQUEST", "detail": "..."}
}
```

---

## Formatting: How AI Should Structure Output

Beyond constraints and examples, you need to tell AI **how to format the output** so it integrates easily into your codebase. The wrong format means copy-paste friction, manual imports, and integration headaches.

Think of it this way: You can ask a contractor to build a deck, but if you don't specify whether you want pre-assembled panels or individual boards, you might get something you can't easily install.

### Three Critical Formatting Options

#### **Option 1: Complete Files (Ready to Copy-Paste)**

Use when: Starting new files or replacing entire files.

**Prompt format**:
```
Formatting: Provide complete, ready-to-use Python file including:
- All necessary imports at the top
- Type hints for all functions
- Google-style docstrings for all functions and classes
- Main execution block (if __name__ == "__main__":)
- Complete error handling
- Example usage in docstrings

Output as a single file I can copy directly to src/validators.py
```

**What you get**: A complete `.py` file you can copy-paste directly into your project without editing imports or adding missing pieces.

**When to use**:
- Creating new modules
- Replacing deprecated files
- Building standalone scripts
- Starting new features from scratch

---

#### **Option 2: Code Snippets with Integration Points**

Use when: Adding functions to existing files or modifying parts of a module.

**Prompt format**:
```
Formatting: Provide code snippet showing:
- Required imports (list separately at top)
- The complete function with docstring and type hints
- Integration point: "Add this function after the existing validate_email() function in src/utils/validators.py"
- Note any additional dependencies needed in requirements.txt
```

**What you get**: A focused snippet with clear instructions on where it goes, plus a checklist of imports and dependencies.

**When to use**:
- Adding functions to existing modules
- Inserting middleware into an existing stack
- Adding new routes to existing router files
- Extending existing classes

---

#### **Option 3: Diff-Style Changes (For Targeted Modifications)**

Use when: Making precise changes to existing code (refactoring, bug fixes, small improvements).

**Prompt format**:
```
Formatting: Show changes in diff format:
- Use "- " prefix for lines to remove
- Use "+ " prefix for lines to add
- Include 3 lines of context before and after changes
- Show line numbers for reference

This helps me see exactly what changes without replacing the entire file.
```

**What you get**: A git-style diff showing exactly what to add, remove, or modify.

**Example output**:
```diff
  def validate_email(email: str) -> bool:
      """Validate email format using regex."""
-     pattern = r'^[a-zA-Z0-9]+@[a-zA-Z0-9]+\.[a-zA-Z]+$'
+     # More comprehensive email validation pattern
+     pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
      return re.match(pattern, email) is not None
```

**When to use**:
- Refactoring specific functions
- Fixing bugs in existing code
- Small improvements or optimizations
- Code reviews where you want to see changes highlighted

---

### Comparison: Which Format When?

| Scenario | Best Format | Why |
|----------|-------------|-----|
| New feature module | Complete file | No existing code to integrate with |
| Adding endpoint to router | Snippet with integration point | Insert into existing file structure |
| Bug fix in function | Diff-style | Minimal changes, easy review |
| Refactoring class | Complete file OR diff | Complete if wholesale rewrite; diff if targeted |
| CLI script | Complete file | Standalone, self-contained |
| Updating middleware | Snippet with integration point | Slot into middleware stack |
| Security patch | Diff-style | Show exactly what changes for review |

---

### Real Example: Three Formats for the Same Task

**Task**: Add input sanitization to an existing function.

#### **Format 1: Complete File**
```
Formatting: Provide the complete updated src/utils/validators.py file with the sanitization function added. Include all imports, existing functions, and the new sanitize_input() function.
```

**Result**: You get the entire file. Easy if validators.py is small; tedious if it's 500 lines and you're only adding one function.

---

#### **Format 2: Snippet with Integration Point**
```
Formatting: Provide only the new sanitize_input() function with:
- Required imports (if any new ones)
- Complete function with docstring and type hints
- Integration point: Add after validate_email() function in src/utils/validators.py
```

**Result**: 
```python
# Required import (add to top of file):
import html

# Add this function after validate_email() in src/utils/validators.py:

def sanitize_input(user_input: str, max_length: int = 1000) -> str:
    """
    Sanitize user input by escaping HTML and limiting length.
    
    Args:
        user_input: Raw user input string
        max_length: Maximum allowed length (default 1000)
        
    Returns:
        Sanitized string safe for storage and display
        
    Raises:
        ValueError: If input exceeds max_length
    """
    if len(user_input) > max_length:
        raise ValueError(f"Input exceeds maximum length of {max_length}")
    
    # Escape HTML to prevent XSS
    sanitized = html.escape(user_input)
    
    # Strip leading/trailing whitespace
    sanitized = sanitized.strip()
    
    return sanitized
```

**Why better**: You can quickly see what to add without scrolling through an entire file.

---

#### **Format 3: Diff-Style**
```
Formatting: Show what to add using diff format with context lines, so I can see where it fits in the existing file structure.
```

**Result**:
```diff
  def validate_email(email: str) -> bool:
      """Validate email format using regex."""
      pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
      return re.match(pattern, email) is not None

+ def sanitize_input(user_input: str, max_length: int = 1000) -> str:
+     """
+     Sanitize user input by escaping HTML and limiting length.
+     
+     Args:
+         user_input: Raw user input string
+         max_length: Maximum allowed length (default 1000)
+         
+     Returns:
+         Sanitized string safe for storage and display
+         
+     Raises:
+         ValueError: If input exceeds max_length
+     """
+     if len(user_input) > max_length:
+         raise ValueError(f"Input exceeds maximum length of {max_length}")
+     
+     # Escape HTML to prevent XSS
+     sanitized = html.escape(user_input)
+     
+     # Strip leading/trailing whitespace
+     sanitized = sanitized.strip()
+     
+     return sanitized

  def validate_password_strength(password: str) -> dict:
```

**Why better**: Shows exactly where in the file it goes, with surrounding context.

---

## Exercise 1: Add Constraints to a Prompt (10 min)

Task: Start with the basic prompt below. Add at least three constraints (version, security, quality) so the AI output must comply.

Basic prompt:
```
Create a FastAPI endpoint to register a user with email and password.
```

Add constraints (example):
- FastAPI 0.104.0 with Annotated
- Password hashing with bcrypt (cost 12)
- Type hints and Google-style docstrings; tests included

Success criteria: AI’s output meets all specified constraints ✓

---

## Exercise 2: Provide a Style Example (5 min)

Task: Include a short snippet from your project (e.g., error handling or service class pattern). Ask AI to match it when implementing the registration endpoint.

Success criteria: The generated code mirrors your example’s structure and tone ✓

---

## Exercise 3: Compare Formatting Options (10 min)

This hands-on exercise demonstrates how formatting instructions change AI output structure.

**Scenario**: You need to add a rate-limiting function to an existing `middleware.py` file.

### Part A: Request Complete File

Run this prompt:
```
Add a rate-limiting function to check if a user has exceeded 100 requests per minute.

Tech stack: Python 3.11, FastAPI 0.104.0

Formatting: Provide the complete middleware.py file with all imports, existing middleware functions, and the new rate-limiting function.
```

**Observe**: How long is the output? If your middleware.py has 10+ functions, do you want to scroll through all of them?

---

### Part B: Request Snippet with Integration Point

Now run this prompt:
```
Add a rate-limiting function to check if a user has exceeded 100 requests per minute.

Tech stack: Python 3.11, FastAPI 0.104.0

Formatting: Provide only:
1. Required imports (list separately)
2. The complete rate-limiting function with docstring
3. Integration point: Tell me where to add it in middleware.py
4. Any new dependencies for requirements.txt
```

**Observe**: Is this easier to integrate? Can you quickly see what to add without full file context?

---

### Part C: Request Diff-Style Changes

Now run this prompt:
```
Add a rate-limiting function to check if a user has exceeded 100 requests per minute.

Tech stack: Python 3.11, FastAPI 0.104.0

Formatting: Show changes in diff format:
- Use "+ " for new lines
- Include 3 lines of context before/after
- Show where in middleware.py it should be added
```

**Observe**: Does the diff format make it crystal clear what to add and where?

---

### Part D: Analysis and Comparison (5 min)

Fill in this comparison table:

| Format | Pros | Cons | Best Used When |
|--------|------|------|----------------|
| Complete file | Everything included, ready to copy | Long output, hard to review | New files, small files |
| Snippet with integration | Focused, clear instructions | Need to know existing structure | Adding to existing files |
| Diff-style | Precise, review-friendly | Need context of existing code | Bug fixes, small changes |

**Reflection**:
1. Which format was easiest to integrate into an existing file?
2. Which format would you use for a code review?
3. Which format saves the most time when working with large files?

**Success criteria**: You understand when to use each format and can specify it in your prompts ✓

---

## Exercise 4: Combine Constraints + Examples + Formatting (10 min)

Task: Write a single prompt that includes all elements from this lesson:
- Command + Context + Logic + Constraints + Example + Formatting

**Template**:
```
Command: [Clear action verb + specific target]

Context:
- Project: [Your stack and phase]
- Tech: [Versions, frameworks]

Logic:
1. [Step 1]
2. [Step 2]
...

Constraints:
- [Version constraints]
- [Security requirements]
- [Quality standards]

Example Pattern:
[Paste a snippet from your project showing style]

Formatting:
[Complete file | Snippet with integration | Diff-style]
Specify: imports, docstrings, type hints, where it goes
```

**Success criteria**: Your prompt produces production-ready code in the exact format you need, matching your project's style ✓

---

## Try With AI

Goal: See the impact of constraints.

1) Unconstrained run
```
Create a function to connect to a PostgreSQL database and return a connection.
```
Observe: Versions, error handling, and secrets handling may be generic or unsafe.

2) Constrained run
```
Create a Python 3.11 function that returns a SQLAlchemy 2.0 Session for PostgreSQL 15. 
Constraints: No hardcoded secrets; read from environment variables. Add explicit error handling. Include type hints and a docstring. Match this error pattern:

try:
    result = await some_operation()
except DatabaseError as e:
    logger.error(f"Database error in operation: {str(e)}")
    raise HTTPException(status_code=500, detail="Database error occurred")
```

3) Compare
- Which output is safer and closer to your project?
- What constraint had the biggest effect?

Outcome: You practiced turning “good code” into “our code” with explicit guardrails.
