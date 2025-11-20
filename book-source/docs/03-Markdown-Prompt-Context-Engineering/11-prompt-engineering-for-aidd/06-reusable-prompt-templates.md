---
title: "Creating Reusable Prompt Templates"
description: "Capture recurring prompt patterns into reusable templates for consistency and efficiency"
sidebar_label: "Creating Reusable Templates"
sidebar_position: 6
chapter: 10
lesson: 6
duration_minutes: 45
proficiency: "B1"
concepts: 8

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Reusable Prompt Templates"
    proficiency_level: "B1"
    category: "Intelligence Design"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student creates 3 prompt templates for recurring development tasks"

learning_objectives:
  - objective: "Create reusable prompt templates for recurring development tasks"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Design 3 templates (bug fix, refactoring, documentation)"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (prompt templates, pattern recognition, template structure, placeholders, reusability criteria, template documentation, template evolution, cross-project value) at B1 threshold ✓"

differentiation:
  extension_for_advanced: "Analyze Zia Kaukab's 8-Element Framework in depth; create template library with 10+ production-ready templates; study template patterns from Anthropic and OpenAI documentation"
  remedial_for_struggling: "Focus on creating one simple template: Identify a task you do 3+ times, extract common elements, create fill-in-the-blank version, test on new scenario"
---

# Creating Reusable Prompt Templates

You've been writing prompts for five lessons. By now, you've probably noticed something: **you're solving the same types of problems repeatedly**.

- Bug fixes: "Debug this authentication timeout..."
- Documentation: "Create API docs for these endpoints..."
- Refactoring: "Improve this function's readability..."
- Code review: "Review this pull request for..."

Each time, you're re-crafting similar prompts from scratch. You're spending 5-10 minutes writing what should take 30 seconds.

**The pattern**: When you solve the same type of problem 2-3 times, you're ready for a **prompt template**.

This lesson teaches you to capture your best prompt patterns into reusable templates—turning recurring prompting work into fill-in-the-blank forms that ensure consistency and save time.

By the end of this lesson, you'll create three prompt templates for tasks you encounter regularly, and you'll know when a problem is "template-worthy" versus when it needs custom prompting.

---

## Templates Encode Framework Elements

Before defining what templates are, recognize this: **Every good template encodes elements from Zia Kaukab's 8-Element Framework** (from Chapter 12 README):

**Zia's 8 Elements**:
1. Command — Task action verb
2. Context — Background information
3. Logic — Problem-solving approach
4. Roleplay — Persona for AI to adopt
5. Formatting — Output structure
6. Constraints — Limitations
7. Examples — Desired output style
8. Iterative Questions — Refinement guidance

**How templates encode these elements**:

| Zia Element | Template Section | Example |
|-------------|-----------------|---------|
| Command | TASK: | "Debug [COMPONENT] issue" |
| Context | CONTEXT: | Last working version, recent changes |
| Logic | CONSTRAINTS: | What must stay same, what can change |
| Roleplay | [Often implicit] | "Think like a debugger..." |
| Formatting | REQUESTED OUTPUT: | "1. Root cause 2. Proposed fix 3. Tests" |
| Constraints | CONSTRAINTS: | "Preserve API, maintain tests" |
| Examples | [Placeholder guidance] | "Include full error messages (don't paraphrase)" |
| Iterative Questions | USAGE NOTES: | Questions to ask yourself before filling |

**Key insight**: By understanding which elements your template encodes, you can design templates that guide AI toward comprehensive, thoughtful responses—because each element activates different parts of the AI's reasoning.

---

## What Is a Prompt Template?

A **prompt template** is a reusable prompt pattern with placeholders for task-specific details.

**Example**: Bug Fix Template

```markdown
TASK: Debug [COMPONENT_NAME] issue

PROBLEM DESCRIPTION:
Current behavior: [WHAT_HAPPENS_NOW]
Expected behavior: [WHAT_SHOULD_HAPPEN]
Error messages: [ERROR_TEXT_OR_NONE]

CONTEXT:
- Last working version: [VERSION_OR_COMMIT]
- Recent changes: [WHAT_CHANGED_RECENTLY]
- Reproducibility: [ALWAYS/SOMETIMES/RARE]

CONSTRAINTS:
- Preserve existing API/behavior
- Maintain test coverage
- No breaking changes

REQUESTED OUTPUT:
1. Root cause analysis
2. Proposed fix with explanation
3. Test cases to prevent regression
```

**Usage**: Copy template → Fill `[PLACEHOLDERS]` → Paste into AI.

**Benefit**: Ensures you never forget critical context (error messages, reproducibility, constraints). Produces consistent, high-quality debugging prompts every time.

---

## When to Create a Template

**Template Creation Criteria** (All three must be true):

1. **Recurrence**: You've performed this task type 2+ times
2. **Pattern**: The task has consistent structure (same questions asked, same output format)
3. **Value**: Saving this pattern would benefit future you (or your team)

**Examples of template-worthy tasks**:

✅ **Bug fixes** — Recurs constantly, follows diagnostic pattern (symptoms → context → constraints → solution)

✅ **Code review** — Recurs for every PR, consistent checklist (correctness, tests, style, performance)

✅ **API documentation** — Recurs for each endpoint, same structure (purpose, request, response, examples)

✅ **Refactoring** — Recurs whenever code becomes complex, same questions (what changes? what stays? what improves?)

✅ **Git commit messages** — Recurs every commit, consistent format (conventional commits structure)

**Examples of NOT template-worthy** (custom prompting better):

❌ **One-time architecture decisions** — Unique context, not recurring

❌ **Exploratory research** — Open-ended, no consistent structure

❌ **Emergency fixes** — Speed over consistency, context too variable

**Rule of thumb**: If you can describe the task type in 2-3 words and you've done it 2+ times → template-worthy.

---

## Prompt Template Structure

Effective templates follow this structure:

```markdown
[TEMPLATE NAME]

PURPOSE: [Why this template exists, when to use it]

---

TASK: [One-line description with [PLACEHOLDERS]]

[SECTION 1: Problem/Context]
[Structured questions with [PLACEHOLDERS]]

[SECTION 2: Constraints]
[Fixed or variable constraints]

[SECTION 3: Requested Output]
[What AI should produce]

---

USAGE NOTES:
- [Guidance on filling placeholders]
- [Common mistakes to avoid]
```

**Key components**:

1. **PURPOSE** — Helps you/team know when to use this template (not every task)
2. **PLACEHOLDERS** — `[ALL_CAPS_IN_BRACKETS]` clearly mark what needs filling
3. **SECTIONS** — Organize information logically (problem → constraints → output)
4. **USAGE NOTES** — Prevent common template misuse

---

## Template 1: Bug Fix Template

Let's create a comprehensive bug fix template.

```markdown
BUG FIX TEMPLATE

PURPOSE: Debug production issues, unexpected behavior, or errors in existing code.
Use when: Something that worked (or should work) is broken.

---

TASK: Debug [COMPONENT/FEATURE_NAME] issue

PROBLEM DESCRIPTION:
Current behavior: [WHAT_ACTUALLY_HAPPENS]
Expected behavior: [WHAT_SHOULD_HAPPEN]
Error messages/stack trace: [PASTE_ERROR_OR_WRITE_NONE]

REPRODUCIBILITY:
- How to reproduce: [STEPS_TO_TRIGGER_BUG]
- Frequency: [ALWAYS/SOMETIMES/RARE]
- Environments affected: [DEV/STAGING/PRODUCTION]

CONTEXT:
- Last known working state: [VERSION/COMMIT/DATE]
- Recent changes: [WHAT_CHANGED_SINCE_LAST_WORKING]
- Affected users/impact: [WHO_IS_AFFECTED_AND_HOW]

CONSTRAINTS:
- Preserve existing API contracts (no breaking changes)
- Maintain current test coverage
- Fix must work in [ENVIRONMENT_REQUIREMENTS]

REQUESTED OUTPUT:
1. Root cause analysis (why is this happening?)
2. Proposed fix with explanation of changes
3. Test cases to prevent regression
4. Verification steps (how to confirm fix works)

---

USAGE NOTES:
- Fill ALL sections even if "None" (shows you've considered it)
- "Last known working state" helps narrow timeframe for investigation
- Include full error messages (don't paraphrase)
- If intermittent bug, describe pattern (time of day, user actions, data conditions)
```

**Example Usage**:

```markdown
TASK: Debug authentication timeout issue

PROBLEM DESCRIPTION:
Current behavior: Users logged out after 15 minutes of idle time
Expected behavior: Users should stay logged in for 24 hours or until explicit logout
Error messages/stack trace: None (silent logout, user sees login screen)

REPRODUCIBILITY:
- How to reproduce: Login → Wait 15 minutes idle → Attempt any action → Redirected to login
- Frequency: Always
- Environments affected: Production (dev environment works correctly)

CONTEXT:
- Last known working state: Version 2.3.0 (deployed 2024-02-28)
- Recent changes: Updated JWT library from v8.5 to v9.0 (2024-03-01)
- Affected users/impact: All logged-in users, critical UX issue

CONSTRAINTS:
- Preserve existing API contracts (no breaking changes)
- Maintain current test coverage
- Fix must work in production (AWS Lambda environment)

REQUESTED OUTPUT:
1. Root cause analysis (why is this happening?)
2. Proposed fix with explanation of changes
3. Test cases to prevent regression
4. Verification steps (how to confirm fix works)
```

**Result**: AI has everything needed for effective debugging. No guessing about environment, no missing context about recent changes.

---

## Template 2: Code Refactoring Template

```markdown
CODE REFACTORING TEMPLATE

PURPOSE: Improve code quality (readability, maintainability, performance) without changing behavior.
Use when: Code works but is hard to understand, modify, or maintain.

---

TASK: Refactor [FUNCTION/CLASS/MODULE_NAME]

CURRENT ISSUES:
- Problem 1: [WHAT_MAKES_THIS_CODE_DIFFICULT]
- Problem 2: [ANOTHER_ISSUE]
- Problem 3: [ANOTHER_ISSUE_OR_DELETE_IF_NONE]

REFACTORING GOALS:
Primary goal: [READABILITY/PERFORMANCE/MAINTAINABILITY]
Success criteria: [HOW_TO_MEASURE_IMPROVEMENT]

WHAT MUST STAY THE SAME:
- API/function signature: [YES/NO - EXPLAIN]
- Behavior: [MUST_BE_IDENTICAL/MINOR_CHANGES_OK]
- Test coverage: [MUST_PASS_EXISTING_TESTS/CAN_MODIFY]

WHAT CAN CHANGE:
- Internal implementation: [YES/NO]
- Dependencies: [CAN_ADD_REMOVE/MUST_KEEP_CURRENT]
- File structure: [CAN_REORGANIZE/MUST_STAY]

CONSTRAINTS:
- Target cyclomatic complexity: [NUMBER_OR_NO_REQUIREMENT]
- Performance: [MUST_NOT_DEGRADE/CAN_ACCEPT_TRADEOFF]
- Backwards compatibility: [REQUIRED/NOT_REQUIRED]

REQUESTED OUTPUT:
1. Refactoring strategy (what will change and why)
2. Refactored code with explanation of improvements
3. Before/after comparison (show improvement)
4. Migration notes (if API changed)

---

USAGE NOTES:
- Be specific about "CURRENT ISSUES" (not just "code is messy")
- Define success criteria measurably (e.g., "cyclomatic complexity < 8")
- Clarify what CAN vs CANNOT change (prevents AI breaking things)
```

**Example Usage**:

```markdown
TASK: Refactor calculateShippingCost function

CURRENT ISSUES:
- Problem 1: Deeply nested if statements (5 levels) make logic hard to follow
- Problem 2: No clear separation between validation, calculation, and formatting
- Problem 3: Variable names unclear (c, x, tmp don't indicate purpose)

REFACTORING GOALS:
Primary goal: Readability
Success criteria: Cyclomatic complexity reduced to < 8, any developer can understand logic in 2 minutes

WHAT MUST STAY THE SAME:
- API/function signature: YES (same inputs/outputs, no breaking changes)
- Behavior: MUST_BE_IDENTICAL (all existing test cases must pass)
- Test coverage: MUST_PASS_EXISTING_TESTS (63 tests currently passing)

WHAT CAN CHANGE:
- Internal implementation: YES (extract helper functions, change algorithm if behavior identical)
- Dependencies: MUST_KEEP_CURRENT (no new libraries)
- File structure: MUST_STAY (single file, don't split)

CONSTRAINTS:
- Target cyclomatic complexity: < 8
- Performance: MUST_NOT_DEGRADE (currently 0.3ms avg, must stay ≤ 0.5ms)
- Backwards compatibility: REQUIRED (function used in 47 places)

REQUESTED OUTPUT:
1. Refactoring strategy (what will change and why)
2. Refactored code with explanation of improvements
3. Before/after comparison (show complexity reduction)
4. Migration notes (if API changed)
```

---

## Template 3: Documentation Generation Template

```markdown
DOCUMENTATION GENERATION TEMPLATE

PURPOSE: Create developer-facing documentation for code, APIs, or features.
Use when: Code exists and needs explanation for other developers.

---

TASK: Create documentation for [COMPONENT/API/FEATURE_NAME]

AUDIENCE:
- Primary readers: [WHO_WILL_READ_THIS]
- Assumed knowledge: [WHAT_THEY_ALREADY_KNOW]
- Use case: [WHY_THEY_NEED_THIS_DOCUMENTATION]

SCOPE:
- What to document: [SPECIFIC_ITEMS_TO_COVER]
- What to exclude: [RELATED_ITEMS_NOT_IN_SCOPE]

FORMAT:
- Output format: [MARKDOWN/JSDOC/OPENAPI/OTHER]
- Code examples: [LANGUAGE_AND_STYLE]
- Length target: [QUICK_REFERENCE/DETAILED_GUIDE/COMPREHENSIVE]

REQUIRED SECTIONS:
1. [SECTION_NAME_1] — [WHAT_IT_COVERS]
2. [SECTION_NAME_2] — [WHAT_IT_COVERS]
3. [SECTION_NAME_3] — [WHAT_IT_COVERS]

EXAMPLE REQUIREMENTS:
- Include: [WORKING_EXAMPLES/CURL_COMMANDS/CODE_SNIPPETS]
- Style: [COPY_PASTEABLE/CONCEPTUAL/BOTH]

SUCCESS CRITERIA:
Developer should be able to: [WHAT_CAN_THEY_DO_AFTER_READING]

---

USAGE NOTES:
- Define audience specifically (not just "developers")
- List exact sections needed (prevents generic structure)
- Specify example style (conceptual vs copy-pasteable changes depth)
```

**Example Usage**:

```markdown
TASK: Create documentation for User Authentication API

AUDIENCE:
- Primary readers: Frontend developers integrating our API
- Assumed knowledge: Familiar with REST APIs and JWT concepts
- Use case: First-time integration of our auth system into their app

SCOPE:
- What to document: POST /login, POST /logout, POST /refresh endpoints
- What to exclude: Registration, password reset (separate guide)

FORMAT:
- Output format: Markdown
- Code examples: JavaScript (fetch API)
- Length target: Quick reference guide (under 500 words)

REQUIRED SECTIONS:
1. Quick Start — Authentication flow overview (30-second read)
2. Endpoint Reference — Each endpoint with request/response examples
3. Common Patterns — Token storage, attaching to requests, handling expiration

EXAMPLE REQUIREMENTS:
- Include: Copy-pasteable fetch() code for each endpoint
- Style: Working examples with placeholder values

SUCCESS CRITERIA:
Developer should be able to: Integrate authentication without reading backend code or asking questions

---

USAGE NOTES:
- Audience is React/Vue/Angular developers (not backend)
- Examples use fetch API (not axios, not curl)
- Focus on integration, not auth theory
```

---

## Template Evolution: Learn and Improve

Templates aren't static. Each time you use one, you learn:
- What questions you keep answering wrong
- What placeholders you keep forgetting
- What output format works best

**Template Evolution Process**:

1. **Create v1** — Based on 2-3 examples of the task
2. **Use 5 times** — Fill template for real tasks
3. **Notice patterns** — Which placeholders are unclear? Which sections get ignored?
4. **Refine to v2** — Add usage notes, clarify placeholders, reorder sections
5. **Share with team** — Get feedback from others using it
6. **Lock to v3** — Stable version everyone uses consistently

**Example Evolution**:

**Bug Fix Template v1** (original):
```markdown
PROBLEM: [DESCRIBE_BUG]
FIX: [WHAT_TO_DO]
```
*Too vague. AI still guesses about context.*

**Bug Fix Template v2** (after 5 uses):
```markdown
PROBLEM: [DESCRIBE_BUG]
ERROR MESSAGE: [PASTE_HERE]
HOW TO REPRODUCE: [STEPS]
FIX: [WHAT_TO_DO]
```
*Better, but missing constraints and recent changes context.*

**Bug Fix Template v3** (current version, shown earlier):
```markdown
[Full structure with problem, reproducibility, context, constraints, output]
```
*Comprehensive. Consistently produces great AI responses.*

**Key insight**: Templates get better each time you discover what was missing.

---

## Storing and Organizing Templates

**Where to keep templates**:

1. **Personal knowledge base** — Markdown files in `~/prompts/` or similar
2. **Project-specific** — `.prompts/bug-fix.md` in your repository
3. **Team shared** — Wiki, Confluence, Notion (shared team resource)

**Organizational structure**:

```
prompts/
  bug-fix.md
  refactoring.md
  documentation.md
  code-review.md
  commit-message.md
  feature-spec.md
```

**Template file format**:

```markdown
# [Template Name]

**Purpose**: [When to use]
**Last Updated**: 2024-03-15
**Version**: 2.1

---

[TEMPLATE CONTENT]

---

## Changelog
- v2.1 (2024-03-15): Added "Recent changes" to context section
- v2.0 (2024-02-10): Restructured into problem/context/constraints/output
- v1.0 (2024-01-05): Initial version
```

**Benefit**: Version tracking shows template evolution. Changelog helps team understand why sections exist.

---

## Common Template Mistakes

### Mistake 1: Templates Too Generic

❌ **Wrong**:
```markdown
TASK: Do [SOMETHING]
OUTPUT: [RESULTS]
```

*Not a template—just a vague prompt with placeholders.*

✅ **Right**:
```markdown
TASK: Debug [COMPONENT] issue

PROBLEM: [Current vs expected behavior]
CONTEXT: [Recent changes, last working version]
CONSTRAINTS: [What must stay same]
OUTPUT: [Root cause + fix + tests]
```

*Structured sections enforce completeness.*

---

### Mistake 2: Too Many Placeholders

❌ **Wrong**:
```markdown
[TASK_TYPE] for [THING] in [LANGUAGE] using [FRAMEWORK] with [CONSTRAINTS]
targeting [AUDIENCE] in [FORMAT] because [REASON]...
```

*15+ placeholders → too much cognitive load, faster to write custom prompt.*

✅ **Right**:
```markdown
TASK: [TASK_TYPE] for [COMPONENT]

CONSTRAINTS:
- Language/framework: [TECH_STACK]
- Audience: [WHO]

OUTPUT: [FORMAT]
```

*Group related placeholders into sections (reduces cognitive load).*

**Rule of thumb**: If template has >8 placeholders, simplify or split into multiple templates.

---

### Mistake 3: No Usage Guidance

❌ **Wrong**:
```markdown
[Template with placeholders but no notes on how to fill them]
```

*User fills vaguely → AI gives mediocre results → user blames template.*

✅ **Right**:
```markdown
---

USAGE NOTES:
- [COMPONENT]: Be specific (not "the app" but "authentication module")
- [CURRENT_BEHAVIOR]: Describe what you observe, not what you think is wrong
- Include full error messages (don't paraphrase)
```

*Guidance prevents misuse, ensures consistent results.*

---

## What You've Learned

You've learned to recognize when prompting patterns should become reusable templates:

1. **Template criteria** — Recurs 2+ times, consistent structure, future value
2. **Template structure** — Purpose, sections (problem/context/constraints/output), placeholders, usage notes
3. **Three templates** — Bug fix, refactoring, documentation (copy and adapt these)
4. **Template evolution** — v1 from examples → v2 after usage → v3 stable
5. **Organization** — Store templates accessibly (local files, project docs, team wiki)

**Core principle**: If you've solved a problem type twice, you'll solve it again. Capture the pattern into a template to ensure consistency and save future time.

In the next lesson, you'll learn **template selection criteria**: decision frameworks for choosing which template to use (or when to write a custom prompt instead).

---

## Try With AI

Ready to capture recurring prompt patterns into reusable templates that save time and ensure consistency?

**🔍 Explore Template-Worthy Patterns:**
> "Analyze my last 5 development tasks: [list them]. Which tasks follow a consistent pattern (same questions asked, same structure needed)? For each recurring pattern, identify: what stays the same (template structure) vs. what changes (placeholders). Show me which are template-worthy."

**🎯 Practice Template Creation:**
> "I perform [your recurring task] multiple times per week. Help me create a reusable template by asking: What sections does this task always need? What placeholders vary? What usage guidance prevents mistakes? What examples would help? Generate a v1.0 template structure."

**🧪 Test Template vs. Custom Prompt:**
> "Take my Bug Fix Template and a unique architecture redesign task. Show me: (1) when using the template saves time (straightforward bug), (2) when custom prompting is faster (template doesn't fit). Explain the decision criteria for template vs. custom."

**🚀 Apply to Your Workflow:**
> "Help me systematize [your actual recurring task]. I'll describe what I do repeatedly. You ask clarifying questions about structure, placeholders, constraints, and output format. Together we'll create a template I can use next time, with clear usage notes preventing common mistakes."

---
