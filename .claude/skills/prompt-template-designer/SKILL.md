---
name: prompt-template-designer
category: "intelligence-design"
applies_to: ["ai-driven-development", "prompting", "reusable-intelligence"]
required_for: ["content-implementer", "prompt-workflows"]
description: |
  Design reusable prompt templates that encode domain-specific patterns for recurring AI development tasks.
  Use this skill when you've executed similar prompts 2+ times and need to capture the pattern as reusable
  intelligence. Transforms one-off prompts into parameterized templates that maintain quality while reducing
  cognitive load. Helps students move from Layer 2 (AI Collaboration) to Layer 3 (Intelligence Design) in
  the 4-Layer Teaching Method.
version: "1.0.0"
dependencies: ["constitution:v6.0.1", "4-layer-teaching-method"]
---

# Prompt Template Designer

## Purpose

Enable developers to transform recurring prompt patterns into reusable templates that accumulate project intelligence. This skill helps:
- Identify when prompts should become templates (2+ uses, 5+ decision points)
- Extract invariant patterns from successful one-off prompts
- Design parameterized templates that encode domain knowledge
- Structure templates using Intent → Constraints → Success Criteria anatomy
- Create template libraries that reduce prompt-writing cognitive load
- Enable consistent quality across team members using same templates

## When to Activate

Use this skill when:
- You've written similar prompts 2+ times (pattern recurs)
- Prompt has 5+ decision points that must be remembered (high cognitive load)
- Team needs consistent prompting patterns (shared quality standards)
- Onboarding new developers to AI-driven workflow
- Building reusable intelligence library for project/domain
- Students ask: "How do I make this prompt reusable?" or "Can I save this pattern?"
- You're teaching Chapter 10 (Prompt Engineering) Lessons 7-8 (Template creation)

## Persona

"Think like a patterns library designer extracting recurring prompt structures into reusable templates. Your goal is to identify what varies (parameters) vs what stays constant (pattern), encode domain knowledge into constraint sets, and design templates that reduce cognitive load while maintaining quality. You're not just storing prompts—you're accumulating intelligence that makes future AI interactions faster and better."

## Questions (Analysis Framework)

Before creating a prompt template, analyze through these questions:

### Question 1: Recurrence Check
**"Have I used this prompt pattern 2+ times?"**
- **If YES**: Pattern recurs, consider template creation
- **If NO**: Keep as one-off prompt (don't prematurely template)
- **Examples**:
  - "Generate Git commit message" (done daily) → YES, create template
  - "Explain React hooks to new developer" (done once) → NO, one-off prompt
  - "Debug Bash permission error" (recurring pattern) → YES, create template
  - "Research specific API quirk" (unique context) → NO, one-off prompt

**Guiding principle**: Premature templating adds maintenance burden. Wait for second use before creating template.

---

### Question 2: Variation Analysis
**"What stays constant vs what changes across uses?"**

Identify **invariants** (template structure) and **variants** (parameters):

**Invariants (template constants)**:
- Intent structure (same action verb, similar goal)
- Core constraints (always required)
- Quality standards (consistent validation criteria)
- Output format (same structure expected)

**Variants (template parameters)**:
- Specific content to process (file names, code snippets)
- Domain context (project-specific details)
- Success thresholds (specific numbers, dates)
- Optional constraints (sometimes needed, sometimes not)

**Example - Git Commit Message**:
```
INVARIANTS:
- Action: GENERATE
- Format: Conventional Commits (<type>(<scope>): <description>)
- Constraint: Imperative mood, <50 char subject
- Validation: Passes commitlint

VARIANTS:
- Changes made (parameter: {{CHANGES}})
- Jira ticket (parameter: {{TICKET_ID}})
- Business value explanation (parameter: {{WHY}})
```

---

### Question 3: Complexity Check
**"Does this prompt have 5+ decision points?"**

Count decisions required to write prompt correctly:

**High-value template indicators** (5+ decisions):
1. Which action verb? (CREATE vs DEBUG vs REFACTOR vs...)
2. What technical constraints apply?
3. What format is required?
4. What validation criteria matter?
5. What edge cases need handling?
6. What success threshold applies?
7. What domain context is essential?

**Example - Code Review Prompt**:
```
Decision points: 8
1. Action verb: ANALYZE
2. Code language: Bash/Python/TypeScript/etc
3. Review focus: Security/Performance/Readability/All
4. Output format: Markdown bullets
5. Severity levels: Critical/Major/Minor
6. Code standards: Which style guide?
7. Context: Is this prod code or prototype?
8. Validation: Are fixes suggested or just identified?

→ 8 decisions = HIGH template value
```

**Low-value template** (1-3 decisions):
```
Decision points: 2
1. Action verb: EXPLAIN
2. Topic: Specific Git command

→ 2 decisions = LOW template value (just ask directly)
```

**Guiding principle**: Cognitive load justifies template when 5+ decisions must be remembered.

---

### Question 4: Domain Knowledge Check
**"Does this template encode domain-specific intelligence?"**

**High-value templates** capture domain patterns:
- **Team conventions** (Jira ticket format, commit structure)
- **Quality standards** (security checks, performance thresholds)
- **Project constraints** (tech stack, architecture patterns)
- **Best practices** (error handling, naming conventions)

**Low-value templates** are generic:
- "Ask AI a question"
- "Generate code"
- "Explain something"

**Example - High Value (Domain-Specific)**:
```markdown
# Backend API Endpoint Template (Domain: E-commerce Platform)

GENERATE new REST API endpoint for {{RESOURCE}}

CONSTRAINTS:
- MUST use Express.js + TypeScript (team standard)
- MUST implement rate limiting (security requirement)
- MUST include Joi validation (validation standard)
- MUST log to structured logger (observability standard)
- MUST include OpenAPI documentation (API doc standard)
- MUST handle currency in cents (domain: avoid floating point errors)
- MUST validate payment methods against allowed list (domain: fraud prevention)

SUCCESS CRITERIA:
- Passes TypeScript compiler (strict mode)
- Passes unit tests (>80% coverage)
- Passes integration tests (test/integration/{{RESOURCE}}.test.ts)
- OpenAPI schema validates (swagger validate)
- Security audit passes (npm audit)
```

**This template encodes**:
- Team tech stack (Express + TypeScript)
- Security patterns (rate limiting, validation)
- Observability patterns (structured logging)
- Domain knowledge (currency handling, payment validation)

---

### Question 5: Parameter Design
**"What parameters make this template flexible but not vague?"**

**Well-designed parameters**:
- **Specific enough** to constrain scope (not "{{INPUT}}" but "{{FILE_PATH}}")
- **Descriptive names** explain what's needed (not "{{X}}" but "{{CHANGES_MADE}}")
- **Clear examples** show valid values
- **Type hints** indicate format (date, path, enum, text)

**Parameter design patterns**:

**Pattern 1: Enum Parameters** (constrained choices)
```markdown
{{ACTION_TYPE}}
Values: [CREATE, DEBUG, REFACTOR, OPTIMIZE, ANALYZE]
Example: CREATE
```

**Pattern 2: Path Parameters** (file/directory references)
```markdown
{{TARGET_FILE}}
Type: file_path
Example: src/utils/validation.ts
```

**Pattern 3: Text Parameters** (free-form but scoped)
```markdown
{{CHANGES_MADE}}
Type: bullet_list
Example:
- Added JWT refresh endpoint
- Extended token expiration to 24h
- Fixed logout race condition
```

**Pattern 4: Composite Parameters** (structured input)
```markdown
{{ERROR_CONTEXT}}
Required fields:
- error_message: string (exact error text)
- file_location: path (where error occurs)
- reproduction_steps: list (how to trigger)
- expected_behavior: string (what should happen)
```

---

### Question 6: Template Organization
**"How should I organize templates for discoverability?"**

**Organization strategies**:

**By Development Phase**:
```
templates/
├── 01-requirements/
│   ├── feature-spec-generator.md
│   └── acceptance-criteria-builder.md
├── 02-implementation/
│   ├── code-generator.md
│   └── refactoring-guide.md
├── 03-testing/
│   ├── test-case-generator.md
│   └── bug-debugger.md
└── 04-documentation/
    ├── readme-writer.md
    └── api-doc-generator.md
```

**By Action Verb** (systematic categorization):
```
templates/
├── create/
│   ├── new-feature.md
│   ├── new-test-suite.md
│   └── new-documentation.md
├── debug/
│   ├── permission-errors.md
│   ├── import-errors.md
│   └── performance-issues.md
└── refactor/
    ├── extract-function.md
    └── simplify-conditionals.md
```

**By Domain** (project-specific):
```
templates/
├── backend-api/
│   ├── new-endpoint.md
│   └── middleware-generator.md
├── frontend-components/
│   ├── new-component.md
│   └── state-management.md
└── devops/
    ├── ci-pipeline.md
    └── deployment-script.md
```

**Guiding principle**: Choose organization that matches team's mental model (how they think about finding templates).

---

## Principles

### Principle 1: Template When Recurs 2+
**"First time is a prompt. Second time is a pattern."**

Don't create templates prematurely:
- **First use**: Write prompt directly, observe what works
- **Second use**: Notice pattern, extract template structure
- **Third+ uses**: Use template, refine based on variations

**Anti-pattern**: Creating template library before having real prompts to extract patterns from.

**Example workflow**:
```
Week 1: Write commit message prompt manually
Week 2: Write similar prompt, notice pattern
Week 2: Create template from common structure
Week 3+: Use template, adjust parameters
```

---

### Principle 2: Parameters Over Hard-Coding
**"Make it reusable by identifying what varies."**

Template design process:
1. **Start with successful prompt** (one that worked well)
2. **Highlight what changed** between uses
3. **Replace specifics with parameters**
4. **Add parameter descriptions**

**Before (hard-coded)**:
```
DEBUG backup.sh script failing with "Permission denied" error

CONSTRAINTS:
- Script runs as user 'developer'
- Destination /backups owned by root:admin
```

**After (parameterized)**:
```
DEBUG {{SCRIPT_NAME}} failing with "{{ERROR_MESSAGE}}" error

CONSTRAINTS:
- Script runs as user '{{RUN_AS_USER}}'
- Destination {{TARGET_PATH}} owned by {{OWNER}}:{{GROUP}}
```

**Parameter guide**:
```
{{SCRIPT_NAME}}: File name of failing script
Example: backup.sh

{{ERROR_MESSAGE}}: Exact error text from logs
Example: Permission denied

{{RUN_AS_USER}}: Unix user running the script
Example: developer

{{TARGET_PATH}}: Directory path where error occurs
Example: /backups

{{OWNER}}: Owner of target directory
Example: root

{{GROUP}}: Group of target directory
Example: admin
```

---

### Principle 3: Document Success Patterns
**"Capture not just structure, but what makes it work."**

Templates should include:
- **When to use** (trigger conditions)
- **Why it works** (what pattern encodes)
- **Example filled** (concrete instantiation)
- **Common mistakes** (what to avoid)
- **Iteration history** (how template evolved)

**Template metadata example**:
```markdown
# Git Commit Message Generator

**When to use**: After staging changes, before committing
**Pattern encoded**: Conventional Commits format + team Jira convention + business value focus
**Success rate**: 95% (from 60% with generic prompts)
**Created**: 2025-10-15
**Last updated**: 2025-11-18
**Iterations**: 7 (refined based on team feedback)
**Usage count**: 47 commits across 3 developers

**Common mistakes**:
- Forgetting to include Jira ticket ID
- Writing "what" instead of "why" in body
- Using past tense instead of imperative mood

**Example usage**:
[See filled template below]
```

---

### Principle 4: Version and Evolve Templates
**"Templates improve through use."**

Template lifecycle:
1. **v1.0**: Initial extraction from successful prompts
2. **v1.1**: Add constraints discovered through edge cases
3. **v1.2**: Refine success criteria based on output quality
4. **v2.0**: Restructure based on team feedback

**Versioning strategy**:
```markdown
## Version History

### v2.0.0 (2025-11-18)
- BREAKING: Changed parameter {{CHANGES}} to structured {{CHANGES_MADE}} with required fields
- Added success criterion: "Business value explained for each change"
- Removed constraint: "Subject line <50 chars" (now in commitlint, not prompt)

### v1.2.0 (2025-11-10)
- Added parameter: {{JIRA_TICKET}}
- Added constraint: "Include ticket from branch name"

### v1.0.0 (2025-10-15)
- Initial template extraction from successful commit message prompts
```

**Guiding principle**: Templates are living intelligence, not static documents. Update based on team learnings.

---

### Principle 5: Measure Template Effectiveness
**"Track whether templates improve quality."**

**Metrics to track**:
- **Success rate**: % of outputs meeting success criteria (target: >85%)
- **Time saved**: Prompt writing time (template vs manual)
- **Iteration count**: Avg refinements needed (target: <3 iterations)
- **Adoption rate**: % team uses template vs writes manual prompts

**Example measurement**:
```markdown
# Backend API Endpoint Template - Effectiveness Report

**Period**: Oct 2025 - Nov 2025
**Usage**: 23 endpoints created using template

**Metrics**:
- Success rate: 91% (21/23 passed code review first try)
- Time saved: ~15 min per endpoint (45 min manual → 30 min template)
- Avg iterations: 2.3 (vs 5.7 for manual prompts)
- Adoption: 78% (18/23 devs use template)

**Failure modes** (2 endpoints failed review):
1. Forgot to parameterize database connection (now added to template)
2. Missing rate limiting on public endpoint (now in constraints)

**Template updated to v1.3.0 addressing both failures**
```

---

## Template Structure (Standard Format)

Every template should follow this structure:

```markdown
---
template_name: {{descriptive-name}}
category: {{create|debug|refactor|optimize|analyze|generate|explain|validate}}
domain: {{backend|frontend|devops|testing|documentation|general}}
complexity: {{simple|intermediate|advanced}}
version: {{semantic-version}}
created: {{YYYY-MM-DD}}
last_updated: {{YYYY-MM-DD}}
success_rate: {{percentage}}
usage_count: {{number}}
---

# {{Template Name}}

## When to Use
{{Trigger conditions and use cases}}

## Pattern Encoded
{{What domain knowledge this captures}}

## Parameters

### {{PARAMETER_NAME_1}}
- **Type**: {{enum|path|text|number|date|composite}}
- **Description**: {{What this represents}}
- **Example**: {{Concrete value}}
- **Required**: {{yes|no}}

### {{PARAMETER_NAME_2}}
...

## Template

\```
INTENT:
{{ACTION_VERB}} {{description with parameters}}

CONSTRAINTS:
- {{invariant constraint 1}}
- {{invariant constraint 2}}
- {{parameterized constraint using {{PARAM}}}}

SUCCESS CRITERIA:
- {{measurable criterion 1}}
- {{measurable criterion 2}}
- {{parameterized criterion using {{PARAM}}}}
\```

## Example (Filled)

\```
{{Concrete instantiation with real values}}
\```

## Common Mistakes
- {{Anti-pattern 1 and how to avoid}}
- {{Anti-pattern 2 and how to avoid}}

## Iteration History
{{Version changelog showing evolution}}
```

---

## Example Template: Git Commit Message Generator

```markdown
---
template_name: git-commit-message-conventional
category: generate
domain: general
complexity: simple
version: 2.0.0
created: 2025-10-15
last_updated: 2025-11-18
success_rate: 95%
usage_count: 47
---

# Git Commit Message Generator (Conventional Commits)

## When to Use
After staging changes (git add), before committing. Use when you need a commit message that:
- Follows Conventional Commits format
- Includes team's Jira ticket convention
- Explains business value (not just technical changes)

## Pattern Encoded
- Conventional Commits standard (<type>(<scope>): <description>)
- Team's Jira ticket format requirement
- Business value focus (why this matters to users/team)
- Imperative mood convention

## Parameters

### CHANGES_MADE
- **Type**: composite (list of changes with context)
- **Description**: List of changes made in this commit with technical details
- **Example**:
  ```
  - Added JWT refresh endpoint (/auth/refresh)
  - Extended token expiration from 1h to 24h
  - Fixed logout race condition in session handler
  ```
- **Required**: yes

### JIRA_TICKET
- **Type**: text (format: PROJ-NNNN)
- **Description**: Jira ticket ID from current branch name
- **Example**: PROJ-1234
- **Required**: yes (use "NO-TICKET" if no ticket exists)

### SCOPE
- **Type**: enum [auth, api, ui, db, devops, docs, test]
- **Description**: Area of codebase affected
- **Example**: auth
- **Required**: yes

### TYPE
- **Type**: enum [feat, fix, docs, refactor, test, chore, perf]
- **Description**: Type of change being made
- **Example**: feat
- **Required**: yes

## Template

\```
GENERATE Git commit message using Conventional Commits format

CHANGES MADE:
{{CHANGES_MADE}}

CONSTRAINTS:
- Format: {{TYPE}}({{SCOPE}}): <description> [{{JIRA_TICKET}}]
- Subject line: Imperative mood, <50 characters
- Body: Bullet list explaining WHY (business value, not just technical what)
- Include Jira ticket in brackets at end of subject line
- Each change in body should explain impact/value

SUCCESS CRITERIA:
- Passes commitlint validation
- Subject ≤50 characters
- Body explains business value for each change
- Teammate can understand changes without reading diff
- Follows team's commit message style
\```

## Example (Filled)

\```
GENERATE Git commit message using Conventional Commits format

CHANGES MADE:
- Added JWT refresh endpoint (/auth/refresh)
- Extended token expiration from 1h to 24h
- Fixed logout race condition in session handler

CONSTRAINTS:
- Format: feat(auth): <description> [PROJ-1234]
- Subject line: Imperative mood, <50 characters
- Body: Bullet list explaining WHY (business value, not just technical what)
- Include Jira ticket in brackets at end of subject line
- Each change in body should explain impact/value

SUCCESS CRITERIA:
- Passes commitlint validation
- Subject ≤50 characters
- Body explains business value for each change
- Teammate can understand changes without reading diff
- Follows team's commit message style

AI Response:
---
feat(auth): add JWT refresh and extend token lifetime [PROJ-1234]

- Add /auth/refresh endpoint: improves mobile UX by eliminating frequent re-logins
- Extend token expiration to 24h: reduces authentication friction for daily users
- Fix logout race condition: ensures users are fully logged out (security improvement)
---
\```

## Common Mistakes
- **Forgetting Jira ticket**: Always include ticket ID even if "NO-TICKET"
- **Using past tense**: "Added feature" → Should be "Add feature" (imperative mood)
- **Explaining WHAT instead of WHY**: "Adds endpoint" → Should explain value: "Improves UX by..."
- **Vague descriptions**: "Update auth" → Be specific: "Add JWT refresh endpoint"

## Iteration History

### v2.0.0 (2025-11-18)
- Changed CHANGES parameter to CHANGES_MADE with structured format
- Added explicit business value requirement in constraints
- Added "Teammate can understand without diff" success criterion

### v1.2.0 (2025-11-10)
- Added JIRA_TICKET parameter
- Added "Include ticket from branch name" constraint

### v1.0.0 (2025-10-15)
- Initial template from successful commit message patterns
```

---

## Teaching Integration (Layer 2 → Layer 3 Transition)

**Use this skill to guide students from AI Collaboration (L2) to Intelligence Design (L3)**:

### Transition Signal
**When student has**:
- Written similar prompts 2+ times
- Demonstrated prompt anatomy understanding (Intent/Constraints/Success)
- Successfully iterated prompts to 85%+ quality

**Activate template creation**:
```
"You've written great commit message prompts twice now. Notice the pattern?
Let's extract this as a reusable template so you don't have to remember all
those constraints every time. We'll identify what stays constant (the pattern)
vs what changes (the parameters)."
```

### Teaching Sequence
1. **Identify recurrence**: "You've done this before. What's similar?"
2. **Extract constants**: "What constraints appear in both prompts?"
3. **Parameterize variants**: "What changed between the two uses?"
4. **Document pattern**: "Why does this structure work?"
5. **Create template**: "Let's formalize this as reusable intelligence"

### Success Indicator
Student can:
- Recognize when to template (2+ uses, 5+ decisions)
- Extract parameters vs constants
- Use their own template successfully
- Explain template value: "This saves me 10 minutes and ensures I don't forget the Jira ticket requirement"

---

## Anti-Patterns (What NOT to Do)

### Anti-Pattern 1: Template Before Pattern
**Symptom**: Creating templates for prompts you haven't used yet

**Why it fails**: You don't know what works, what varies, or what's essential

**Fix**: Write prompts manually 2+ times, then extract template from successful patterns

---

### Anti-Pattern 2: Over-Parameterization
**Symptom**: Template has 15+ parameters

**Why it fails**: Cognitive load of filling template exceeds benefit

**Fix**: Aim for 3-7 parameters. If more needed, consider splitting into multiple focused templates

---

### Anti-Pattern 3: Under-Parameterization (Too Specific)
**Symptom**: Template only works for one specific scenario

**Why it fails**: Not reusable; just a saved prompt, not a pattern

**Fix**: Ensure template applies to at least 3+ concrete use cases

---

### Anti-Pattern 4: No Success Metrics
**Symptom**: Created template but never tracked if it improves quality

**Why it fails**: Can't tell if template is valuable or needs refinement

**Fix**: Track success rate, time saved, iteration count after template adoption

---

## Quick Reference

**Template Creation Checklist**:
- [ ] Used prompt pattern 2+ times?
- [ ] Identified constants vs variants?
- [ ] Counted 5+ decision points?
- [ ] Designed clear parameters with examples?
- [ ] Included filled example?
- [ ] Documented when to use?
- [ ] Added success criteria?
- [ ] Planned version tracking?

**Parameter Design Checklist**:
- [ ] Descriptive names (not {{X}} or {{INPUT}})?
- [ ] Type specified (enum/path/text/composite)?
- [ ] Examples provided?
- [ ] Required vs optional marked?

**Template Quality Check**:
- [ ] Success rate >85%?
- [ ] Time savings measurable?
- [ ] Team adoption >50%?
- [ ] Evolved through iterations?
