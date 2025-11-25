---
sidebar_position: 6
title: "Skill Creation + Polish"
---

# Skill Creation + Polish

You've built four features in less time than Feature 1 alone consumed. That acceleration wasn't luck. You solved the same problem patterns multiple times: JSON transformations appeared in Features 1, 2, 3, and 4. The specification structure you refined for Lead Profiler shaped every feature that followed. Error handling approaches you developed once got reused four times without rethinking.

This is intelligence accumulation. Patterns that repeat 2+ times with 5+ decision points become skills. Skills encode knowledge so future projects benefit immediately—no re-solving, no re-discovering, no re-deciding.

Now you'll formalize those patterns into actual reusable skills using the **P+Q+P Framework: Persona + Questions + Principles**. You'll create real skill files you can invoke in future projects.

## Identify Recurring Patterns

Review your four feature specs and implementations. What patterns appeared in 2 or more features?

**Common patterns in sales pipelines**:
- **JSON data transformer**: Input JSON → processing → structured JSON output
- **Specification structure**: Input/Output/Success Criteria/Error Cases template
- **Pipeline connector**: Output of Feature N feeds input of Feature N+1
- **CLI formatter**: Terminal output (tables, lists, summaries)
- **Error handler**: Validation, missing fields, graceful failure
- **Data validator**: Schema validation before transformation

**List your top 2-3 recurring patterns**:

1. _____________________
2. _____________________
3. (optional) _____________________

## Create Your First Skill File

Choose your most valuable pattern—the one you want every future project to inherit.

Create the file:

```bash
mkdir -p .claude/skills
touch .claude/skills/[your-pattern-name].md
```

Write the complete skill. Use this structure:

```markdown
# Skill: [Pattern Name]

## Persona
You are [role]. You [value system]. You prioritize [decision criteria].

## Questions (Ask Before Implementing)
1. [Key design question 1]
2. [Key design question 2]
3. [Key design question 3]
4. [Key design question 4]
5. [Key design question 5]

## Principles (Apply During Implementation)
- **[Principle 1 name]**: [Why this matters]
- **[Principle 2 name]**: [Why this matters]
- **[Principle 3 name]**: [Why this matters]
- **[Principle 4 name]**: [Why this matters]
- **[Principle 5 name]**: [Why this matters]

## Example Application
[Real example from one of your 4 features showing how you applied this skill]
```

**Example**: If your pattern is "JSON Data Transformer", your skill might look like:

```markdown
# Skill: JSON Data Transformer

## Persona
You are a data pipeline architect who transforms JSON inputs into structured JSON outputs.
You prioritize schema consistency, graceful error handling, and clear field documentation.

## Questions (Ask Before Implementing)
1. What is the input JSON schema? (required fields, optional fields, nested structures?)
2. What is the output JSON schema? (exact field names, types, nested structure?)
3. What happens when required input fields are missing or invalid?
4. Should transformation preserve original fields or only output derived fields?
5. What validation should occur before transformation? (type checking, range validation?)

## Principles (Apply During Implementation)
- **Schema First**: Define output schema before writing transformation logic
- **Fail Explicitly**: Missing required fields → error with clear message, not silent default
- **Preserve Context**: Include source reference or metadata in output when relevant
- **Type Safety**: Validate input types match expected schema before processing
- **Test Edge Cases**: Empty arrays, null values, missing optional fields, boundary values

## Example Application
When building ICP Scorer (Feature 2), you took Lead Profile JSON as input (company_name, industry, size, tech_indicators)
and output Score JSON (score 0-100, category, breakdown, reasoning).
You applied "Fail Explicitly" when required fields were missing, returning score 0 with reasoning "incomplete profile".
You applied "Schema First" by writing the output schema before the scoring logic.
```

**Create your actual skill file now** before proceeding.

## Create Your Second Skill File

Choose another pattern—ideally different from your first. If your first skill is data-focused (JSON transformer), your second might be structural (spec writing) or operational (error handling).

```bash
touch .claude/skills/[your-second-pattern-name].md
```

Write the complete skill using the same P+Q+P structure: Persona, 5+ Questions, 5+ Principles, Example Application.

**Example candidates**:
- **Feature Specification Writer**: Patterns in how you structured specs for each feature
- **Error Handler**: How you handled validation, missing fields, edge cases
- **Data Validator**: Schema validation approach used across features
- **CLI Output Formatter**: Terminal display patterns (tables, lists, summaries)

**Create your second skill file now.**

## Test Skills Against Feature 5

Your skills should guide someone else (or future-you) through building Feature 5 without re-deciding design patterns.

**Feature 5: Lead Nurture Sequencer**

Scenario: After outreach sent, no response in 3 days → recommend next action with timing and message variant.

Input: Combined lead data (lead profile + ICP score + outreach history)

Output: Recommendation (action type, timing, message variant)

### Part 1: Apply Your First Skill

Using your first skill's Questions and Principles, answer:
- What input schema does Feature 5 need? (List fields)
- What validation should happen before processing?
- What output schema makes sense?
- What edge cases exist? (no outreach history? multiple interactions?)

**Document your answers** (2-3 sentences per question).

### Part 2: Apply Your Second Skill

Using your second skill's Questions and Principles, answer:
- What triggers Feature 5? (time elapsed? event?)
- What data must it receive? (input schema)
- What does it produce? (output schema)
- How do we know it worked? (success criteria)
- What could go wrong? (error cases)

**Document your answers.**

### Part 3: Evaluate Completeness

After applying your skills:
- Did the Questions force you to think about design decisions you'd overlooked?
- Did the Principles guide your implementation choices?
- What additional Question or Principle would strengthen the skill?

If the skill feels incomplete, refine it now.

## Create Your Third Skill (Optional)

If momentum exists and patterns remain, create one more skill. Candidates from your four features:
- **Pipeline Connector**: How output of one feature becomes input for the next
- **Specification Template**: Consistent structure across all four specs
- **API/CLI Interface**: How features expose inputs and outputs
- **Logging and Debugging**: Error messages, diagnostics, traces

**Optional**: Create your third skill file.

## Try With AI

Test your skills with AI feedback:

**Prompt 1**: "Review this skill definition: [paste your first skill]. Does the P+Q+P structure cover the design decisions needed to implement this pattern again? What Questions or Principles am I missing? What should I remove because it's too specific?"

**Prompt 2**: "I'm building a new 4-feature sales pipeline (different domain—B2B hiring instead of sales). Given my skills [skill-name-1] and [skill-name-2], what would you recommend as a third skill to create? What pattern did I likely use in my sales pipeline that would transfer to hiring?"
