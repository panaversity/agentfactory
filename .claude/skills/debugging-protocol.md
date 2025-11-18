# debugging-protocol

**Description**: Systematic debugging protocol for isolating root cause through hypothesis testing, applicable to any domain (markdown, bash, git, Python, APIs, systems).

**When to use this skill**: Apply when encountering errors, unexpected behavior, or configuration issues that require systematic diagnosis rather than trial-and-error fixes.

---

## Persona

"Think like a diagnostician isolating root cause through hypothesis testing, not random trial-and-error"

**Cognitive stance**: Debugging is NOT guessing fixes. It's systematic isolation of cause from symptom through evidence-based hypothesis testing. Prioritize understanding WHY over quick fixes.

---

## Questions

Apply these 5 questions sequentially when debugging any issue:

### 1. What EXACTLY is the symptom?

**Purpose**: Make the problem observable, measurable, and reproducible.

**Ask**:
- What specific behavior occurs? (Not "doesn't work", but "produces X when I expect Y")
- Can you reproduce it reliably? (If not, symptom is not yet isolated)
- What's the minimal example that shows the problem? (Reduce to simplest case)
- What are the exact error messages or unexpected outputs?

**Bad symptom**: "My markdown is broken"
**Good symptom**: "Second-level list items with 4-space indentation render as code blocks instead of list items"

---

### 2. What could cause this symptom?

**Purpose**: Generate ranked hypotheses based on probability, not guesswork.

**Ask**:
- What are 3-5 possible root causes?
- Which causes are most likely? (Common configuration/syntax issues first)
- Which causes are less likely? (Tool-specific limitations, edge cases)
- What assumptions am I making? (Are they valid?)

**Ranking criteria**:
- **High probability**: Common causes (configuration, syntax, indentation, typos)
- **Medium probability**: Tool-specific issues (parser limitations, version incompatibilities)
- **Low probability**: Rare edge cases (encoding, OS-specific, hardware)

**Example**:
- H1 (most likely): Indentation is 4 spaces (triggers code block) instead of 2 spaces (list standard)
- H2 (likely): Mixing tabs and spaces in indentation
- H3 (less likely): Markdown parser doesn't support nested lists in this context
- H4 (unlikely): File encoding issue (UTF-8 vs ASCII)

---

### 3. What's the simplest test to confirm/refute hypothesis?

**Purpose**: Design minimal experiments that provide evidence, not complex multi-variable changes.

**Ask**:
- What's the SMALLEST change that tests this hypothesis?
- What outcome confirms the hypothesis? What outcome refutes it?
- Am I changing only ONE variable at a time? (If not, simplify test)
- How will I measure the result? (Observable output, error message, behavior change)

**Bad test**: "Rewrite the entire section and see if it works"
**Good test**: "Change indentation from 4 spaces to 2 spaces in one list item and check rendering"

**Test design pattern**:
```
Hypothesis: [Specific cause]
Test: [Minimal change]
Expected result if TRUE: [Observable outcome]
Expected result if FALSE: [Different observable outcome]
```

---

### 4. Did the fix resolve symptom without introducing new issues?

**Purpose**: Validate that the fix addresses root cause and doesn't create regression.

**Ask**:
- Is the original symptom completely resolved? (Not just hidden)
- Did this introduce any new issues? (Regression check)
- Can I explain WHY this fix works? (Root cause understanding)
- Is this fix generalizable? (Does it apply to similar cases?)

**Validation checklist**:
- ✅ Original symptom resolved
- ✅ No new issues introduced
- ✅ Root cause understood (can explain WHY)
- ✅ Fix applies to all similar cases (not just one instance)

**If validation fails**: Return to Question 2 (hypothesis generation) with new information.

---

### 5. What did I learn that applies to future debugging?

**Purpose**: Extract reusable insights for future troubleshooting.

**Ask**:
- What was the root cause category? (Syntax, configuration, environment, logic)
- What diagnostic technique worked best? (Reading logs, minimal testing, comparison)
- What would I do differently next time? (Earlier isolation, better hypothesis ranking)
- Does this pattern recur enough to encode as a skill? (2+ occurrences, 5+ decision points)

**Learning capture pattern**:
```
Root cause: [What was actually wrong]
Diagnostic insight: [What technique helped isolate it]
Future application: [How to recognize this pattern faster]
```

---

## Principles

These are decision frameworks, not rigid rules. Apply judgment to context.

### Principle 1: Isolation Before Fixing

**Framework**: "Never attempt fixes until symptom is specific and reproducible. Vague symptoms lead to random fixes that mask root cause."

**What this means**:
- If symptom is vague ("doesn't work"), spend time isolating it first
- Create minimal reproducible examples (MRE) before hypothesizing
- Document exact error messages, line numbers, unexpected outputs

**Example**:
- ❌ Bad: "Git push fails" → Try random git commands
- ✅ Good: "Git push rejected with 'non-fast-forward' error" → Hypothesis: local branch behind remote

---

### Principle 2: Hypothesis Ranking

**Framework**: "Test high-probability causes first. Don't debug rare edge cases until common causes are eliminated."

**What this means**:
- Start with syntax/configuration issues (80% of bugs)
- Then check tool-specific limitations (15% of bugs)
- Finally investigate rare edge cases (5% of bugs)

**Avoid**: Immediately jumping to exotic explanations (encoding, OS, hardware) before checking common issues (typos, indentation, missing dependencies).

---

### Principle 3: Evidence-Based Validation

**Framework**: "Confirm hypotheses with tests, don't assume. If you can't test it, you can't trust it."

**What this means**:
- Design tests that produce observable outcomes
- Compare expected vs actual results
- Change ONE variable at a time to isolate cause

**Avoid**: Changing multiple things simultaneously and claiming success when something works (you don't know WHAT fixed it).

---

### Principle 4: Regression Prevention

**Framework**: "Validate that fixes don't break other functionality. Fix should improve one thing without degrading others."

**What this means**:
- Test fix in original context (does it solve the symptom?)
- Test fix in adjacent contexts (does it break similar cases?)
- Run existing tests (if available) to catch regressions

**Example**: Fixed nested lists with 2-space indentation → Test all list types (ordered, unordered, nested) to ensure nothing broke.

---

### Principle 5: Root Cause Understanding

**Framework**: "If you can't explain WHY the fix works, understanding is incomplete. Symptoms might reappear."

**What this means**:
- Don't accept fixes that "just work" without understanding
- Research the underlying mechanism (why does 4-space indentation trigger code blocks?)
- Document root cause for future reference

**Avoid**: Cargo-cult debugging ("I copied this fix from StackOverflow, it works, but I don't know why").

---

## Reusability

**This skill transfers across**:
- **Markdown rendering** (syntax/formatting errors)
- **Bash scripts** (command/environment errors)
- **Git workflows** (state/history errors)
- **Python errors** (syntax/runtime/logic errors)
- **API debugging** (request/response/authentication errors)
- **System architecture** (configuration/networking/performance issues)

**Evidence of reusability** (tested in Chapter 10, Lesson 6):
- ✅ Debugged markdown lists (indentation issue)
- ✅ Debugged bash script (command not found)
- ✅ Debugged git workflow (non-fast-forward error)

**Future application** (Part 4, Python debugging):
- Apply to Python syntax errors, runtime exceptions, logic bugs
- Same 5 questions, same principles, different domain
- Protocol is domain-agnostic (reasoning pattern, not tool-specific)

---

## Usage Example

**Scenario**: Python script fails with "ModuleNotFoundError: No module named 'requests'"

**Apply debugging-protocol**:

**Question 1: What EXACTLY is the symptom?**
- Symptom: Script exits with "ModuleNotFoundError: No module named 'requests'" on line 3
- Reproducible: Yes (happens every time script runs)
- Minimal example: `import requests` line triggers error

**Question 2: What could cause this?**
- H1 (most likely): requests library not installed in current environment
- H2 (likely): Running script with different Python interpreter than where requests is installed
- H3 (less likely): Typo in import statement
- H4 (unlikely): File system permissions blocking module loading

**Question 3: What's the simplest test?**
- Test H1: Run `pip list | grep requests` to check if installed
- Expected if TRUE: No output (not installed)
- Expected if FALSE: `requests  2.28.0` (installed)

**Question 4: Did fix resolve symptom?**
- Fix: `pip install requests`
- Validation: Script runs without error ✅
- No regression: Other imports still work ✅
- Root cause understood: requests library was missing from environment ✅

**Question 5: What did I learn?**
- Root cause: Missing dependency
- Diagnostic insight: Check `pip list` before assuming code error
- Future application: Always verify environment dependencies match requirements.txt

---

---

## Transfer Validation

**This skill claims to transfer to Python development (Part 4, Chapters 12-29).**

**Validation checkpoint**: When Part 4 is implemented, validate that the Persona + Questions + Principles pattern works for Python debugging without modification:

**Test cases**:
1. **Syntax errors**: Does Question 1 (What EXACTLY is the symptom?) isolate Python syntax errors effectively?
2. **Runtime exceptions**: Does Question 2 (What could cause this?) generate ranked hypotheses for AttributeError, TypeError, KeyError?
3. **Logic bugs**: Does Question 3 (What's the simplest test?) design minimal tests for incorrect calculations or control flow?
4. **Import errors**: Does the protocol work for ModuleNotFoundError and circular imports?
5. **Type errors**: Does hypothesis ranking work for type hint violations and Pydantic validation errors?

**Expected outcome**: Protocol works without Python-specific modifications. If Python-specific adjustments are needed (e.g., "Check type hints" as common hypothesis), document them as extensions, not replacements.

**Validation date**: [To be completed when Part 4 Python chapters are implemented]

---

## Version

**Version**: 1.0.0
**Created**: 2025-01-18 (Chapter 10, Lesson 6)
**Domain**: Cross-domain debugging (markdown, bash, git, Python, APIs, systems)
**Pattern**: Persona + Questions + Principles (reasoning-activated)
**Constitution**: v6.0.0 Compliance
