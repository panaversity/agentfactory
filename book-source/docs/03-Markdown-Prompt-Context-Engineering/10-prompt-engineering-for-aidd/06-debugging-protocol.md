# Lesson 6: Systematic Debugging Protocol — Creating Reusable Intelligence

---

## What Should This Lesson Enable You to Do?

By the end of this lesson, you should be able to:

- **Apply a systematic debugging protocol** (isolate → hypothesize → test → validate) to any error
- **Recognize when a pattern recurs enough** to justify creating reusable intelligence (2+ occurrences, 5+ decision points)
- **Create a prompt skill** using Persona + Questions + Principles pattern
- **Test skills for reusability** across multiple domains (markdown, bash, git → will transfer to Python)
- **Understand the 4-layer context model** in depth (how AI uses layered context for reasoning)

**Success criteria**: You can debug a markdown rendering issue using systematic protocol, encode that protocol as a reusable skill, and explain why this skill will transfer to Python debugging in Part 4.

---

## The Systematic Debugging Problem

### The Scenario

You are a **Developer Relations Engineer** creating tutorial content for your company's API. You've written a markdown file with code examples, but when you render it:

**Problem**: Lists aren't displaying correctly. Some items appear as paragraphs, some as nested lists, and the formatting is inconsistent.

**Your initial approach** (trial-and-error):
```
"AI, fix my markdown lists"
AI suggests: "Check for indentation issues"
You try: Add spaces randomly
Result: Some lists fix, others break
You try: Remove all indentation
Result: Different errors appear
You try: Copy someone else's markdown
Result: Still broken in your context
```

After 30 minutes, you're frustrated. **This is reactive debugging** — no system, no learning, no reusable insights.

---

### The Systematic Approach

What if instead, you applied a **diagnostic protocol** like a doctor diagnosing symptoms?

1. **Isolate**: What EXACTLY is broken? (Observable, measurable symptom)
2. **Hypothesize**: What could cause this? (Ranked list of likely causes)
3. **Test**: What's the simplest test to confirm/refute hypothesis?
4. **Validate**: Did the fix resolve symptom without introducing new issues?

**This lesson teaches you that protocol and encodes it as reusable intelligence.**

---

## Concept 1: The 4-Step Debugging Protocol

### Step 1: Isolate the Symptom (Make It Observable)

**Vague symptom** (unusable):
> "My markdown is broken"

**Specific symptom** (actionable):
> "Nested lists render as paragraphs when indented with 4 spaces, but render correctly with 2 spaces"

**How to isolate**:

**Question 1**: What is the EXACT behavior? (Not "doesn't work", but "produces X when I expect Y")

**Question 2**: Can you reproduce it reliably? (If not, symptom is not yet isolated)

**Question 3**: What's the minimal example that shows the problem? (Reduce to simplest case)

**Example**: Your markdown lists issue

**Vague**: "Lists broken"
**Specific**: "Second-level list items (indented 4 spaces) render as code blocks instead of list items"
**Minimal example**:
```markdown
- Item 1
    - Subitem 1 (renders as code, not list)
```

**Now you have an isolated, reproducible symptom.**

---

### Step 2: Hypothesize Possible Causes (Ranked by Likelihood)

Once you have a specific symptom, generate hypotheses ranked by probability.

**For the markdown lists issue, hypotheses might be**:

**Hypothesis 1** (most likely): Indentation is 4 spaces (code block threshold) instead of 2 spaces (list indentation standard)

**Hypothesis 2** (likely): Mixing tabs and spaces in indentation

**Hypothesis 3** (less likely): Markdown parser doesn't support nested lists in this context

**Hypothesis 4** (unlikely): File encoding issue (UTF-8 vs ASCII)

**How to rank hypotheses**:
- **High probability**: Common causes (configuration, syntax, indentation)
- **Medium probability**: Tool-specific issues (parser limitations)
- **Low probability**: Rare edge cases (encoding, OS-specific)

**Strategic insight**: Test high-probability hypotheses first. Don't debug rare edge cases until common causes are eliminated.

---

### Step 3: Test Hypothesis with Minimal Experiment

For each hypothesis, design the **simplest test** that confirms or refutes it.

**Hypothesis 1**: 4 spaces is triggering code block rendering

**Test**: Change indentation from 4 spaces to 2 spaces in minimal example
```markdown
# Before (4 spaces)
- Item 1
    - Subitem 1

# After (2 spaces)
- Item 1
  - Subitem 1
```

**Expected result**:
- If hypothesis TRUE: Subitem renders as list item
- If hypothesis FALSE: Still renders incorrectly

**Test**: Run markdown through renderer, check output

**Result**: ✅ Subitem now renders correctly

**Conclusion**: Hypothesis 1 confirmed. 4-space indentation was the root cause.

---

**Hypothesis 2** (if Hypothesis 1 fails): Mixing tabs and spaces

**Test**: Convert all indentation to spaces (or tabs) consistently
```bash
# Reveal tabs vs spaces
cat -A file.md

# Convert tabs to spaces
expand -t 2 file.md > file_fixed.md
```

**Expected result**:
- If hypothesis TRUE: Consistent indentation fixes rendering
- If hypothesis FALSE: Still broken

---

### Step 4: Validate Fix Doesn't Introduce Regression

After fixing, **validate comprehensively**:

**Validation checklist**:
1. ✅ Original symptom resolved? (Nested lists render correctly)
2. ✅ No new issues introduced? (Other lists still work)
3. ✅ Root cause understood? (Can you explain WHY 4 spaces broke it?)
4. ✅ Generalizable fix? (Does this apply to all nested lists in this file?)

**Testing validation**:
```markdown
# Test multiple cases
- Item 1
  - Subitem 1 ✅
  - Subitem 2 ✅
    - Nested Subitem ✅
- Item 2
  - Subitem A ✅
```

**If all cases pass → Fix validated.**

---

## Concept 2: When to Create Reusable Intelligence (Skill Design)

### Recognizing Reusability Signals

You've now debugged markdown lists using the 4-step protocol. **But should you encode this as a reusable skill?**

**Ask these questions**:

**Question 1**: Have I encountered this pattern 2+ times?
- Markdown debugging: Yes (lists, tables, code blocks, links)
- Specific to markdown lists: No (only once)

**Question 2**: Does this pattern have 5+ decision points?
- Debugging protocol: Yes (isolate, rank hypotheses, design tests, validate, iterate)
- Specific markdown list fix: No (just one decision)

**Question 3**: Does this transfer across domains?
- Debugging protocol: YES (markdown → bash → git → Python → any troubleshooting)
- Specific markdown fix: No (only applies to markdown lists)

**Conclusion**: Create a reusable **debugging-protocol** skill (not a markdown-specific skill).

---

### The Reusable Skill Pattern: Persona + Questions + Principles

Every reusable skill has three components:

**1. Persona**: "Think like [role]" — The cognitive stance
**2. Questions**: 5 analytical questions guiding decision-making
**3. Principles**: Decision frameworks (not rigid rules)

**For debugging-protocol skill**:

---

**Persona**:
> "Think like a diagnostician isolating root cause through hypothesis testing, not random trial-and-error"

Why this matters: Debugging is NOT guessing fixes. It's systematic isolation of cause from symptom.

---

**Questions**:

1. **What EXACTLY is the symptom?** (Observable, measurable, reproducible)
2. **What could cause this symptom?** (Hypothesis list, ranked by probability)
3. **What's the simplest test to confirm/refute hypothesis?** (Minimal experiment)
4. **Did the fix resolve symptom without introducing new issues?** (Validation + regression check)
5. **Can I explain WHY this fix works?** (Root cause understanding)

---

**Principles**:

1. **Isolation before fixing**: Don't attempt fixes until symptom is specific and reproducible
2. **Hypothesis ranking**: Test high-probability causes first (common issues before rare edge cases)
3. **Evidence-based validation**: Confirm hypothesis with tests, don't assume
4. **Regression prevention**: Validate fix doesn't break other functionality
5. **Root cause understanding**: If you can't explain WHY the fix works, understanding is incomplete

---

## Concept 3: Testing Skill Reusability Across Domains

### Why Test Across Domains?

A truly reusable skill works beyond the original context. **Testing reusability** validates that your skill isn't overfitted to one scenario.

**For debugging-protocol skill, test across**:

1. **Markdown debugging** (original domain)
2. **Bash script debugging** (different domain, same protocol)
3. **Git workflow debugging** (different domain, same protocol)

If the skill works across all three → Transfers to Python debugging (Part 4).

---

### Domain Test 1: Markdown Lists (Original Context)

**Symptom**: Nested lists render as code blocks

**Apply debugging-protocol**:
1. **Isolate**: Second-level items with 4-space indentation render as code
2. **Hypothesize**: 4 spaces triggers code block (H1), tabs mixed (H2), parser issue (H3)
3. **Test**: Change to 2-space indentation
4. **Validate**: All nested lists now render correctly, no other lists broken

**Result**: ✅ Protocol successfully debugged markdown lists

---

### Domain Test 2: Bash Script Errors (Different Context)

**Symptom**: Bash script fails with "command not found: python"

**Apply debugging-protocol**:
1. **Isolate**: Script runs `python script.py`, but system has `python3` not `python`
2. **Hypothesize**: Python alias issue (H1), PATH missing (H2), Python not installed (H3)
3. **Test**: Change command to `python3 script.py`
4. **Validate**: Script runs successfully, no other commands broken

**Result**: ✅ Protocol successfully debugged bash script

---

### Domain Test 3: Git Workflow Issues (Different Context)

**Symptom**: `git push` rejected with "non-fast-forward" error

**Apply debugging-protocol**:
1. **Isolate**: Local branch is behind remote (someone pushed changes)
2. **Hypothesize**: Need to pull first (H1), force push (H2), branch mismatch (H3)
3. **Test**: Run `git pull --rebase` then `git push`
4. **Validate**: Push succeeds, no commits lost, history clean

**Result**: ✅ Protocol successfully debugged git workflow

---

### Reusability Validation

**The debugging-protocol skill worked across**:
- Markdown rendering (syntax/formatting errors)
- Bash scripts (command/environment errors)
- Git workflows (state/history errors)

**Conclusion**: This skill is **domain-agnostic** and will transfer to:
- Python errors (Part 4, Chapters 12-29)
- API debugging (Part 5)
- System architecture issues (Part 6)

**This is Stage 3 Intelligence Design** — encoding patterns that recur across contexts.

---

## Concept 4: The 4-Layer Context Model (Deep Dive)

### Why This Matters for Debugging

In Lesson 3, you learned the 4-layer context model (Project → Code → Constraints → Analyst). **For debugging**, these layers provide critical context for hypothesis generation.

**Without layered context**, AI guesses generic causes:
> "Check for syntax errors, indentation, or file encoding"

**With layered context**, AI generates domain-specific hypotheses:
> "Given your markdown renderer (Docusaurus), 4-space indentation triggers code blocks. Standard nested list indentation is 2 spaces. Test changing indentation."

---

### Layer 1: Project Context (The "Why")

**What AI needs to know**: Why are you debugging this? What's the impact?

**Example**:
```markdown
## Project Context
- Creating API tutorial content for developer onboarding
- Markdown files render on Docusaurus documentation site
- Impact: Broken lists confuse new developers, reduce onboarding success
- Timeline: Publishing tutorial tomorrow (time pressure)
```

**How this helps debugging**:
- AI prioritizes fixes that work with Docusaurus specifically
- AI suggests workarounds if full fix takes too long
- AI considers impact (onboarding blockers) in hypothesis ranking

---

### Layer 2: Code Context (The "What")

**What AI needs to know**: What exactly is broken?

**Example**:
```markdown
## Code Context
- File: docs/tutorials/api-basics.md
- Section: "Nested List Examples" (lines 45-60)
- Symptom: Second-level list items render as code blocks
- Minimal example: [paste minimal broken example]
```

**How this helps debugging**:
- AI focuses on specific file/section (not entire codebase)
- AI can analyze minimal example to isolate cause
- AI avoids suggesting fixes for unrelated issues

---

### Layer 3: Constraints Context (The "How")

**What AI needs to know**: What are the limitations?

**Example**:
```markdown
## Constraints Context
- Markdown parser: Docusaurus (not GitHub Flavored Markdown)
- Cannot change parser (company standard)
- Cannot use custom CSS (policy restriction)
- Must maintain accessibility (screen reader compatibility)
```

**How this helps debugging**:
- AI suggests fixes that work within Docusaurus markdown spec
- AI avoids suggesting parser changes or CSS hacks
- AI considers accessibility in fix design

---

### Layer 4: Analyst Context (The "Who")

**What AI needs to know**: Who is debugging, and what's their background?

**Example**:
```markdown
## Analyst Context
- Role: Developer Relations Engineer
- Background: 3 years markdown, 5 years documentation, 1 month Docusaurus
- Gaps: Not expert in markdown parser internals
- Need: Explanation of WHY fix works (for future reference)
```

**How this helps debugging**:
- AI explains at appropriate level (not too basic, not too expert)
- AI provides rationale (WHY), not just solution (WHAT)
- AI considers your learning goal (future reference)

---

### Applying 4-Layer Context to Debugging

**Prompt with 4-layer context**:
```
## Project Context
Creating API tutorial, publishing tomorrow, Docusaurus site

## Code Context
File: api-basics.md, lines 45-60, nested lists render as code blocks
Minimal example:
- Item 1
    - Subitem 1 (renders as code)

## Constraints Context
Must use Docusaurus markdown, cannot change parser, no custom CSS

## Analyst Context
Developer Relations Engineer, markdown experience but not Docusaurus expert,
need explanation of WHY fix works

Apply debugging protocol: isolate → hypothesize → test → validate
```

**AI response** (with layered context):
```
## Isolation
Symptom: Second-level items with 4-space indentation render as code blocks

## Hypothesis (ranked by Docusaurus-specific likelihood)
H1 (most likely): Docusaurus uses 2-space nested list standard, 4 spaces
    triggers code block (consistent with CommonMark spec)
H2 (less likely): Mixing tabs and spaces
H3 (unlikely): File encoding issue

## Test
Change indentation from 4 spaces to 2 spaces:
- Item 1
  - Subitem 1

## Validation
Check: Does 2-space indentation render as list item in Docusaurus?
Expected: Yes (CommonMark standard)

## Explanation (WHY this works)
Docusaurus follows CommonMark spec:
- 2 spaces = nested list item
- 4 spaces = code block (indented code)
Your original 4-space indentation accidentally triggered code block rendering.

This is a Docusaurus-specific behavior (some parsers allow 3-4 spaces for lists).
For future reference: Always use 2 spaces for nested lists in Docusaurus.
```

**Notice how 4-layer context shaped the response**:
- Project context: AI prioritized quick fix (publishing tomorrow)
- Code context: AI analyzed minimal example
- Constraints context: AI suggested Docusaurus-compatible fix
- Analyst context: AI explained WHY (not just WHAT)

---

## Self-Assessment: Debugging Protocol Practice

### Exercise 1: Isolate Symptoms

For each vague symptom, write a **specific, observable, reproducible** version.

**Vague A**: "My bash script doesn't work"
- **Specific**: ___________

**Vague B**: "Git is broken"
- **Specific**: ___________

**Vague C**: "Markdown table looks weird"
- **Specific**: ___________

---

### Exercise 2: Hypothesis Ranking

**Symptom**: Python script fails with "ModuleNotFoundError: No module named 'requests'"

**Rank these hypotheses** (1 = most likely, 4 = least likely):
- ___ requests library not installed
- ___ Python version incompatibility
- ___ Typo in import statement ("requets" instead of "requests")
- ___ File system permissions issue

---

### Exercise 3: Design Minimal Tests

**Symptom**: Docusaurus build fails with "Duplicate anchor ID" error

**Design 3 tests** (ordered by hypothesis likelihood):
1. Test: ___________
2. Test: ___________
3. Test: ___________

---

### Answer Key (Self-Check)

**Exercise 1**:
- A: "Bash script exits with 'command not found: python' on line 5"
- B: "`git push` rejected with 'non-fast-forward' error"
- C: "Markdown table has 4 columns defined but 5 data cells in row 2, causing misalignment"

**Exercise 2**:
1. requests library not installed (most common cause)
2. Typo in import statement (easy to check, common)
3. Python version incompatibility (less likely, would show different error)
4. File system permissions (rare, specific error message)

**Exercise 3**:
1. Test: Grep for duplicate heading IDs in markdown files
2. Test: Check if any headings have identical text (auto-generated IDs)
3. Test: Validate no manual anchor tags have duplicate IDs

---

## Try With AI

**Setup**: Open Claude Code or Gemini CLI. Have a markdown file with intentional errors ready (or create one with broken lists, tables, or code blocks).

**Exercise**: Apply the debugging protocol using AI collaboration.

---

### Practice Task: Systematic Debugging

**Goal**: Debug a markdown rendering issue using the 4-step protocol.

**Prompts to Try**:

**Prompt 1: Isolate Symptom**
```
I have a markdown rendering issue. Here's the file: @broken-markdown.md

Help me isolate the symptom:
1. What EXACTLY is broken? (Not "lists don't work", but specific observable behavior)
2. Can you identify a minimal example that reproduces it?
3. What's the specific line/section where the issue occurs?
```

**Expected outcome**: AI should identify specific symptom (e.g., "Line 23: Nested list with 4-space indentation renders as code block instead of list item").

---

**Prompt 2: Generate Hypotheses**
```
Symptom: [paste specific symptom from Prompt 1]

Apply debugging protocol:
1. What are 3 possible causes, ranked by likelihood?
2. For each hypothesis, explain why it's likely/unlikely
3. Suggest the simplest test for the most likely hypothesis
```

**Expected outcome**: AI should provide ranked hypotheses with reasoning (not just guesses).

---

**Prompt 3: Test and Validate**
```
I tested Hypothesis 1 by [describe what you changed].

Result: [describe outcome]

Did this confirm or refute the hypothesis?
If confirmed: How do I validate the fix doesn't break other sections?
If refuted: What's the next hypothesis to test?
```

**Expected outcome**: AI should analyze test results and guide next steps systematically.

---

### Challenge: Create Your Own Debugging Skill

**Goal**: Encode your debugging experience as a reusable skill.

**Steps**:

1. **Identify recurring pattern**: Have you debugged similar issues 2+ times?
2. **Extract decision points**: What questions do you ask each time?
3. **Write skill structure**:

```markdown
# debugging-protocol

**Persona**: "Think like [your debugging approach]"

**Questions**:
1. [Your Question 1]
2. [Your Question 2]
3. [Your Question 3]
4. [Your Question 4]
5. [Your Question 5]

**Principles**:
1. [Your Principle 1]
2. [Your Principle 2]
3. [Your Principle 3]
```

4. **Test across domains**: Does this work for markdown, bash, git?
5. **Validate reusability**: Could you use this skill in Python debugging?

---

### Safety Note

**Validate debugging outputs**: Always verify that:
- **Symptom isolation** is specific enough to act on
- **Hypotheses** are ranked by actual likelihood (not random guesses)
- **Tests** are minimal (don't change multiple variables at once)
- **Fixes** don't introduce regression (test other functionality)

AI can guide debugging systematically, but you remain responsible for validating that fixes solve the root cause.

---

**Lesson Metadata**:
- **Stage**: 3 (Intelligence Design)
- **Modality**: Error analysis + skill creation
- **Concepts**: 4 (4-step debugging protocol, skill creation workflow, reusability testing, 4-layer context deep dive)
- **Cognitive Load**: B1 tier (4 ≤ 10)
- **AI Tools**: Claude Code OR Gemini CLI (platform-agnostic)
- **Duration**: 80 minutes
- **Skill Created**: debugging-protocol (Persona + Questions + Principles)
- **Version**: 1.0.0
- **Constitution**: v6.0.0 Compliance
- **Generated**: 2025-01-18
- **Source Spec**: `specs/025-chapter-10-redesign/spec.md`
