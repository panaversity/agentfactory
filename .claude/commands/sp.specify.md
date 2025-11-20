---
description: Create or update the feature specification from a natural language feature description.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

The text the user typed after `/sp.specify` in the triggering message **is** the feature description. Assume you always have it available in this conversation even if `$ARGUMENTS` appears literally below. Do not ask the user to repeat it unless they provided an empty command.

Given that feature description, do this:

0. **Detect existing feature branch** (for git worktree workflows):

   a. Check current branch:
   
      CURRENT_BRANCH=$(git branch --show-current 2>/dev/null)

      # Check if branch matches: NNN-name or feature-NNN-name
      if [[ "$CURRENT_BRANCH" =~ ^([0-9]+-|feature-[0-9]+-).*$ ]]; then
        echo "EXISTING_FEATURE_BRANCH_DETECTED"
      fi

   b. If existing feature branch detected:
      - Extract feature number and short-name from branch name
      - Set `FEATURE_DIR=specs/{number}-{name}`
      - Set `SPEC_FILE={FEATURE_DIR}/spec.md`
      - **Skip steps 1-2** (branch already exists)
      - **Proceed to step 3** (load spec template)

      Example:
      - Current branch: `feature-001-upload`
      - Extract: number=001, name=upload
      - Create spec at: `specs/001-upload/spec.md`
      - Do NOT create new branch

   c. If NO existing feature branch:
      - Proceed normally with steps 1-2 (create new branch)
      
1. **Generate a concise short name** (2-4 words) for the branch:
   - Analyze the feature description and extract the most meaningful keywords
   - Create a 2-4 word short name that captures the essence of the feature
   - Use action-noun format when possible (e.g., "add-user-auth", "fix-payment-bug")
   - Preserve technical terms and acronyms (OAuth2, API, JWT, etc.)
   - Keep it concise but descriptive enough to understand the feature at a glance
   - Examples:
     - "I want to add user authentication" → "user-auth"
     - "Implement OAuth2 integration for the API" → "oauth2-api-integration"
     - "Create a dashboard for analytics" → "analytics-dashboard"
     - "Fix payment processing timeout bug" → "fix-payment-timeout"

2. **Check for existing branches before creating new one**:

   **If step 0 detected existing branch**:
   - Skip this entire step (branch already exists)
   - Proceed to step 3

   **Otherwise** (standard workflow):
   
   a. First, fetch all remote branches to ensure we have the latest information:
      ```bash
      git fetch --all --prune
      ```
   
   b. Find the highest feature number across all sources (globally):
      - Remote branches: `git ls-remote --heads origin | grep -E 'refs/heads/[0-9]+-'`
      - Local branches: `git branch | grep -E '^[* ]*[0-9]+-'`
      - Specs directories: Check for directories matching `specs/[0-9]+-*`
      - Archived specs: Check for directories matching `specs/archived/[0-9]+-*`

   c. Determine the next available number:
      - Extract all numbers from all four sources (regardless of short-name)
      - Find the highest number N globally
      - Use N+1 for the new branch number
   
   d. Run the script `.specify/scripts/bash/create-new-feature.sh --json "$ARGUMENTS"` with the calculated number and short-name:
      - Pass `--number N+1` and `--short-name "your-short-name"` along with the feature description
      - Bash example: `.specify/scripts/bash/create-new-feature.sh --json "$ARGUMENTS" --json --number 5 --short-name "user-auth" "Add user authentication"`
      - PowerShell example: `.specify/scripts/bash/create-new-feature.sh --json "$ARGUMENTS" -Json -Number 5 -ShortName "user-auth" "Add user authentication"`
   
   **IMPORTANT**:
   - Check all four sources (remote branches, local branches, specs directories, archived specs) to find the highest number globally
   - Match ALL branches/directories regardless of short-name to get global highest number
   - If no existing branches/directories found anywhere, start with number 1
   - Numbers are assigned sequentially across ALL features (global counter, not per-feature)
   - You must only ever run this script once per feature
   - The JSON is provided in the terminal as output - always refer to it to get the actual content you're looking for
   - The JSON output will contain BRANCH_NAME and SPEC_FILE paths
   - For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot")

3. Load `.specify/templates/spec-template.md` to understand required sections.

4. Follow this execution flow:

    1. Parse user description from Input
       If empty: ERROR "No feature description provided"
    2. Extract key concepts from description
       Identify: actors, actions, data, constraints
    3. For unclear aspects:
       - Make informed guesses based on context and industry standards
       - Only mark with [NEEDS CLARIFICATION: specific question] if:
         - The choice significantly impacts feature scope or user experience
         - Multiple reasonable interpretations exist with different implications
         - No reasonable default exists
       - **LIMIT: Maximum 3 [NEEDS CLARIFICATION] markers total**
       - Prioritize clarifications by impact: scope > security/privacy > user experience > technical details
    4. Fill User Scenarios & Testing section
       If no clear user flow: ERROR "Cannot determine user scenarios"
    5. Generate Functional Requirements
       Each requirement must be testable
       Use reasonable defaults for unspecified details (document assumptions in Assumptions section)
    6. Define Success Criteria
       Create measurable, technology-agnostic outcomes
       Include both quantitative metrics (time, performance, volume) and qualitative measures (user satisfaction, task completion)
       Each criterion must be verifiable without implementation details
    7. Identify Key Entities (if data involved)
    8. Return: SUCCESS (spec ready for planning)

5. Write the specification to SPEC_FILE using the template structure, replacing placeholders with concrete details derived from the feature description (arguments) while preserving section order and headings.

6. **Specification Quality Validation**: Invoke spec-architect subagent to validate specification completeness and quality.

   a. **Invoke spec-architect agent**:

      Use the Task tool to launch spec-architect with this prompt:

      ```
      Validate specification completeness and quality for: {SPEC_FILE}

      Context:
      - Feature directory: {FEATURE_DIR}
      - Feature description: {original user arguments}
      - Spec just written, needs validation before planning

      Your task:
      1. Analyze spec against your reasoning framework:
         - Testability Analysis (are requirements falsifiable?)
         - Completeness Check (constraints, non-goals, edge cases defined?)
         - Ambiguity Detection (would 3 engineers implement identically?)
         - Traceability Mapping (prerequisites, downstream impacts clear?)

      2. Generate quality checklist to: {FEATURE_DIR}/checklists/requirements.md
         Use your standard checklist template covering:
         - Content Quality (no implementation details, user-focused)
         - Requirement Completeness (testable, measurable, technology-agnostic)
         - Feature Readiness (acceptance criteria, user scenarios, scope boundaries)

      3. Identify issues with severity levels:
         - CRITICAL: Blocks planning (vague requirements, missing success criteria)
         - MAJOR: Needs refinement (ambiguous constraints, undefined edge cases)
         - MINOR: Nice to have (clarifying examples, additional context)

      4. If [NEEDS CLARIFICATION] markers exist:
         - Extract all markers from spec
         - Prioritize by impact: scope > security/privacy > UX > technical details
         - Keep ONLY top 3 most critical clarifications
         - For each, generate clarification question with 3-4 option table format
         - Make informed guesses for lower-priority items

      5. If CRITICAL issues found (non-clarification):
         - Provide specific spec improvements (quote line, suggest fix)
         - Apply fixes automatically where unambiguous
         - Maximum 2 auto-fix iterations

      Return structured validation report with:
      - Checklist status (pass/fail per item)
      - Issue list (severity + specific location + suggested fix)
      - Clarification questions (max 3, table format ready for user)
      - Overall readiness verdict (READY / NEEDS_CLARIFICATION / NEEDS_FIXES)
      ```

   b. **Process spec-architect response**:

      - **If verdict is READY**:
        - Checklist auto-generated and all items pass
        - Proceed to step 7 (report completion)

      - **If verdict is NEEDS_FIXES**:
        - Review spec-architect's suggested fixes
        - Apply fixes to spec.md
        - Re-invoke spec-architect for validation (max 1 additional iteration)
        - If still failing after re-validation, warn user with specific remaining issues

      - **If verdict is NEEDS_CLARIFICATION**:
        - spec-architect has generated clarification questions (max 3)
        - Present questions to user in spec-architect's table format
        - Wait for user responses (e.g., "Q1: A, Q2: Custom - [details], Q3: B")
        - Update spec by replacing [NEEDS CLARIFICATION] markers with user answers
        - Re-invoke spec-architect for final validation

   c. **Checklist Management**:
      - spec-architect auto-generates checklist at {FEATURE_DIR}/checklists/requirements.md
      - Updates checklist after each validation iteration
      - Final checklist reflects pass/fail status of all quality criteria

7. Report completion with branch name, spec file path, spec-architect validation verdict, checklist results, and readiness for the next phase (`/sp.clarify` or `/sp.plan`).

**NOTE:** The script creates and checks out the new branch and initializes the spec file before writing.

## General Guidelines

## Quick Guidelines

- Focus on **WHAT** users need and **WHY**.
- Avoid HOW to implement (no tech stack, APIs, code structure).
- Written for business stakeholders, not developers.
- DO NOT create any checklists that are embedded in the spec. That will be a separate command.

### Section Requirements

- **Mandatory sections**: Must be completed for every feature
- **Optional sections**: Include only when relevant to the feature
- When a section doesn't apply, remove it entirely (don't leave as "N/A")

### For AI Generation

When creating this spec from a user prompt:

1. **Make informed guesses**: Use context, industry standards, and common patterns to fill gaps
2. **Document assumptions**: Record reasonable defaults in the Assumptions section
3. **Limit clarifications**: Maximum 3 [NEEDS CLARIFICATION] markers - use only for critical decisions that:
   - Significantly impact feature scope or user experience
   - Have multiple reasonable interpretations with different implications
   - Lack any reasonable default
4. **Prioritize clarifications**: scope > security/privacy > user experience > technical details
5. **Think like a tester**: Every vague requirement should fail the "testable and unambiguous" checklist item
6. **Common areas needing clarification** (only if no reasonable default exists):
   - Feature scope and boundaries (include/exclude specific use cases)
   - User types and permissions (if multiple conflicting interpretations possible)
   - Security/compliance requirements (when legally/financially significant)

**Examples of reasonable defaults** (don't ask about these):

- Data retention: Industry-standard practices for the domain
- Performance targets: Standard web/mobile app expectations unless specified
- Error handling: User-friendly messages with appropriate fallbacks
- Authentication method: Standard session-based or OAuth2 for web apps
- Integration patterns: RESTful APIs unless specified otherwise

### Success Criteria Guidelines

Success criteria must be:

1. **Measurable**: Include specific metrics (time, percentage, count, rate)
2. **Technology-agnostic**: No mention of frameworks, languages, databases, or tools
3. **User-focused**: Describe outcomes from user/business perspective, not system internals
4. **Verifiable**: Can be tested/validated without knowing implementation details

**Good examples**:

- "Users can complete checkout in under 3 minutes"
- "System supports 10,000 concurrent users"
- "95% of searches return results in under 1 second"
- "Task completion rate improves by 40%"

**Bad examples** (implementation-focused):

- "API response time is under 200ms" (too technical, use "Users see results instantly")
- "Database can handle 1000 TPS" (implementation detail, use user-facing metric)
- "React components render efficiently" (framework-specific)
- "Redis cache hit rate above 80%" (technology-specific)

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.
