---
title: "Validating AI-Generated Code — Safety-First Habits"
chapter: 10
lesson: 5
duration_minutes: 45

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Applying a 5-Step Validation Checklist"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student consistently applies the checklist before using AI code in their project"

  - name: "Recognizing Red Flags in Code"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Safety"
    measurable_at_this_level: "Student identifies 3+ red flags (e.g., hardcoded secrets, missing error handling) in sample code"

learning_objectives:
  - objective: "Adopt a validation-first mindset and workflow when using AI-generated code"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Hands-on checklist application to flawed code samples"

  - objective: "Use the 5-step validation checklist to evaluate and iterate on AI output"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Exercise where students apply all 5 steps and prompt AI to fix issues"

  - objective: "Spot common red flags and request targeted fixes from the AI collaborator"
    proficiency_level: "A2"
    bloom_level: "Analyze"
    assessment_method: "Red flag recognition exercise with 4/5 correct threshold"

cognitive_load:
  new_concepts: 2
  assessment: "Exactly 2 new concepts (5-step checklist, red flags) within CEFR A2 scope ✓"

differentiation:
  extension_for_advanced: "Add security scanning (dependency checks, secrets scanning) and propose CI validation gates"
  remedial_for_struggling: "Provide a simplified 3-step version of the checklist and worked examples before independent practice"
---

## What: Validation-First Safety Culture

This is the safety-critical lesson. After Lessons 2–4, you can generate code with strong commands, context, and logic. Now you’ll learn the habit that prevents costly mistakes: validate everything the AI produces before it touches your codebase.

In AI-native development, you are the architect and reviewer. The AI is your collaborator, not an infallible oracle. Professional responsibility means you always verify correctness, security, and alignment with the spec before integrating code.

## Why: Validation Matters

AI coding agents generate answers token-by-token based on patterns. They can produce brilliant solutions—or plausible but unsafe ones. Typical risks include:
- Hallucinated APIs or functions that don’t exist
- Security issues (e.g., raw SQL, weak hashing, exposed secrets)
- Fragile code paths (no error handling, no input validation)
- Mismatch with your constraints (versions, style, tests)

The fix is a lightweight, repeatable checklist you apply every time. It keeps speed while raising quality.

## How: The 5-Step Validation Checklist

Use this sequence for every AI-generated snippet, file, or PR:

1) Read First — sanity and intent check
- Skim top-to-bottom. Does the code do exactly what you asked? Are imports, types, and names consistent?
- Compare structure to your requested approach (pattern, algorithm, design).

2) Check Secrets — never commit sensitive data
- Ensure no hardcoded credentials, API keys, tokens, or connection strings.
- Verify config is read from environment or secrets manager.

3) Check Issues — correctness, safety, and style
- Input validation present; error handling explicit (no bare except).
- Security: parameterized queries; safe cryptography (e.g., bcrypt with cost factor); no PII in logs.
- Code quality: type hints, docstrings, consistent style.

4) Test It — prove behavior
- Run or outline minimal unit tests for happy path, edge cases, and error conditions.
- If you can’t run it yet, at least specify expected I/O examples and invariants.

5) Compare to Spec — alignment and constraints
- Re-read your prompt/spec. Does output meet requirements, versions, and your project’s patterns?
- If not, ask the AI for targeted fixes tied to the missed criteria.

### Common Red Flags to Watch For
- Hardcoded credentials or secrets
- No input validation or missing error handling
- Raw SQL without parameters; unsafe string interpolation
- Weak hashing or insecure crypto defaults
- Undefined symbols, hallucinated APIs, mismatched versions
- Logging sensitive data, lack of type hints or tests

---

## Exercise 1: Apply the Validation Checklist (10 min)

Task: Review the flawed snippet below. Identify at least three issues using the checklist.

Flawed example (deliberately unsafe):
```python
import psycopg2

def connect():
    # WARNING: Deliberately flawed for learning
    conn = psycopg2.connect(
        host="localhost",
        user="admin",
        password="secret123",
        database="app"
    )
    cur = conn.cursor()
    email = input("Enter email: ")
    cur.execute(f"SELECT * FROM users WHERE email = '{email}'")
    return cur.fetchall()
```

Success criteria:
- Student flags 3+ issues (hardcoded secret, SQL injection, no error handling, no typing, no context management, logs or prints of sensitive data if added) ✓

---

## Exercise 2: Red Flag Recognition (5 min)

Task: For each short snippet (conceptual, not to run), name the red flag.

1) `hashlib.md5(password.encode()).hexdigest()`
2) `except:
       pass`
3) `requests.get(url)` used inside a hot loop without timeouts
4) Writing raw exception messages directly to client responses
5) Returning DB models with password fields intact

Success criteria: 4/5 correct ✓

---

## Exercise 3: Validate and Fix (10 min)

Task: Paste an AI-generated function from Lessons 2–4. Apply the checklist, then prompt the AI to fix issues.

Requirements:
- Identify at least two concrete issues
- Write a targeted “Fix” prompt (e.g., “Fix to avoid hardcoded credentials and add error handling”) 
- Re-validate the revised code

Success criteria: Final code passes all 5 checklist steps ✓

---

### Pitfalls to Avoid (and How to Fix Them)

These are common mistakes when working with AI-generated code. Use your 5-step checklist to prevent them.

1) Vague Requirements
- Bad: "Make an API for my app"
- Better: Specify resource, methods, schemas, errors, tests.

2) Missing Technical Context
- Bad: "Debug this code" (no stack trace, no environment)
- Better: Provide error, repro steps, environment, related files.

3) Ignoring Existing Patterns
- Bad: "Add a user profile endpoint" (no style reference)
- Better: Include example route/service and ask AI to match it.

4) No Validation Criteria
- Bad: "Optimize this function"
- Better: Define target latency, constraints, and invariants to preserve.

5) Accepting First Output
- Bad: Shipping initial code without checks
- Better: Apply the 5-step checklist and iterate with targeted fix prompts.

## Try With AI

Goal: Practice validation by improving a database connection implementation.

1) Generate code
Copy/paste to your AI collaborator:
```
Create a Python function to connect to PostgreSQL and fetch all rows from a users table. Keep it simple.
```

2) Validate using the 5 steps
- Read First: imports, names, return type
- Check Secrets: no hardcoded credentials; use env vars
- Check Issues: parameterized queries, error handling, timeouts if HTTP present
- Test It: outline happy/edge/error cases
- Compare to Spec: ensure Python 3.11, SQLAlchemy 2.0 or psycopg2 with parameters, consistent with your project

3) Iterate: Ask for a fix
Example fix prompt:
```
Fix to avoid hardcoded credentials, use environment variables, parameterize the query, and add explicit error handling with clear exceptions. Include type hints and a docstring.
```

4) Integrate: Only after it passes the checklist.

Outcome: You practiced the safety habit that turns fast AI output into trustworthy, production-ready code.

