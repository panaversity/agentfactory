# Clarify Phase

The `/sp.clarify` command systematically identifies gaps, ambiguities, and missing assumptions in specifications before the planning phase. Students learn to recognize specification weaknesses (vague terms, unstated assumptions, incomplete requirements, scope conflicts) and iteratively refine specs through AI feedback, preventing cascading problems during implementation and planning.

## Mental Models

- **Detail Detective Pattern**: `/sp.clarify` acts as a critical reader that identifies gaps you didn't catch—ambiguities that seemed clear in your head but are vague on paper (assumptions about scope, audience, success, terminology, requirements, scenarios).

- **Specification Clarity ↔ Plan Quality**: Vague specifications produce vague plans; precise specifications enable specific implementation tasks. Clarification is the bridge between intent and actionable design.

- **Critical vs. Nice-to-Have**: Distinguish gaps that block planning (audience, citation style, length requirements) from improvements that enhance quality (formatting preferences, timeline details). Prioritize critical gaps upfront; defer nice-to-have clarifications.

- **Iterative Specification Refinement**: Most specifications require 1-2 clarification rounds. Each round surfaces remaining gaps, iteratively improving clarity until the spec is ready for planning.

## Key Patterns

- **Four Gap Categories** (identified by `/sp.clarify`):
  - Ambiguous Terms (undefined concepts like "well-researched," "professional format")
  - Missing Assumptions (unstated scope: citation style, audience, word count, source recency)
  - Incomplete Requirements (unspecified scenarios: how to handle conflicts, what sections contain, revision process)
  - Scope Conflicts (unclear focus: historical vs. current, broad vs. narrow, what's in/out)

- **Critical Gap Recognition**: Test whether planning can proceed without this answer—if planning would make different choices without it, it's critical and should be addressed upfront.

- **Ambiguity Prevention Chain**: Specification clarity → Planning precision → Implementation clarity → Smooth execution. Skipping clarification creates cascading problems: vague planning → confused tasks → rework.

- **AI-Driven Gap Discovery**: `/sp.clarify` systematically analyzes specs and returns categorized feedback; students evaluate each finding and decide to accept, reject, or modify the suggestion.

## Common Mistakes

- **Skipping clarification because "spec looks good"**: Every specification has ambiguities you didn't notice. Clarify surfaces them in minutes rather than hours during implementation rework.

- **Ignoring critical clarifications for later**: Deferring decisions that affect all downstream work (citation style, paper length, audience) means planning proceeds without understanding critical constraints, requiring rework mid-implementation.

- **Accepting all AI suggestions without evaluation**: Not all clarification suggestions improve the spec; some add unnecessary complexity. Evaluate each suggestion before accepting: Is it critical? Does it affect planning? Can it be deferred?

- **Over-clarifying (nice-to-have details)**: Treating every suggestion as critical blocks progress. Some details (formatting tools, aesthetic preferences) don't affect planning and can be deferred or omitted.

## Progression Context

- **Builds on**: Lesson 04 (Specify Phase) — Students have written initial specifications. Clarify phase validates specification quality before moving to planning.

- **Leads to**: Lesson 06 (Plan Phase) — Clear, gap-free specifications enable planning phase to generate precise implementation tasks. Clarification ensures planning has all information needed for design decisions.
