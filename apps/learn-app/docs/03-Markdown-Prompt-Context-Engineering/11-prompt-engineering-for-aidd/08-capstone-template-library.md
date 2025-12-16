---
title: "Capstone: Prompt Template Library Specification"
description: "Write complete specification for a Prompt Template Library tool orchestrating all Chapter 12 concepts"
sidebar_label: "Capstone: Template Library"
sidebar_position: 8
chapter: 10
lesson: 8
duration_minutes: 60
proficiency: "B1"
concepts: 5

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Specification-Driven Prompt Engineering"
    proficiency_level: "B1"
    category: "Spec-Driven Integration"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student writes complete specification for Prompt Template Library tool"

learning_objectives:
  - objective: "Create complete specification for Prompt Template Library tool using Chapter 12 principles"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Peer-reviewable specification document with success criteria and validation tests"

cognitive_load:
  new_concepts: 5
  assessment: "5 concepts (specification integration, template library architecture, version control, template discovery, usage analytics) at B1 threshold ✓"

differentiation:
  extension_for_advanced: "Research existing prompt management tools (PromptBase, LangChain prompts); design advanced features like A/B testing, analytics, and collaborative template sharing; create comprehensive validation framework"
  remedial_for_struggling: "Focus on minimal viable specification: Define core user story (store and retrieve templates), 3 key features, and 3 success criteria before attempting full specification"
---

# Capstone: Prompt Template Library Specification

You've learned eight prompt engineering concepts across this chapter:

1. Prompts as specifications (WHAT, not HOW)
2. Prompt anatomy (Intent → Constraints → Success Criteria)
3. Iterative refinement (60% → 97% through iteration)
4. Specification-first prompting (define success before prompting)
5. Question-Driven Development (AI asks clarifying questions)
6. Reusable templates (capture recurring patterns)
7. Template selection (decision frameworks for template vs custom)

**Now it's time to orchestrate these concepts into a complete project.**

This capstone lesson asks you to write a **specification document** for a Prompt Template Library tool—a system that stores, organizes, and helps developers select prompt templates for recurring tasks.

**You will NOT implement this tool** (that would require Python, which you learn in Part 4). Instead, you'll write the **specification** that defines WHAT this tool should do, HOW it should behave, and WHAT success looks like.

This specification will be **peer-reviewable**: another developer (or AI) should be able to implement the tool from your spec without asking clarification questions.

By the end of this lesson, you'll have a complete specification artifact demonstrating your mastery of prompt engineering as specification writing.

---

## What Is a Prompt Template Library?

A **Prompt Template Library** is a tool (CLI, web app, or IDE extension) that:

1. **Stores** prompt templates in organized structure
2. **Discovers** which template fits a given task
3. **Guides** developers through filling template placeholders
4. **Validates** filled templates for completeness
5. **Learns** from usage to recommend better templates over time

**Real-world analogy**: Like code snippet libraries (GitHub Gists, SnippetBox, VS Code snippets), but for **AI prompts** instead of code.

**Example user workflow**:

```bash
$ prompt-library select "fix authentication bug"

🔍 Found 2 matching templates:
  1. Bug Fix Template (general debugging)
  2. Authentication Bug Template (auth-specific)

Select template: 2

📝 Fill template placeholders:

[COMPONENT]: authentication middleware
[CURRENT_BEHAVIOR]: users logged out after 15 min
[EXPECTED_BEHAVIOR]: stay logged in 24 hours
[ERROR_MESSAGE]: None

✓ Template complete. Copy to clipboard? (Y/n) Y

📋 Copied to clipboard. Paste into AI assistant.
```

**Your task**: Write the specification that defines how this tool works.

---

## Your Capstone Task

**Write a specification document** for a Prompt Template Library tool that helps developers:
1. Store and organize their prompt templates
2. Quickly find the right template for their task
3. Fill templates correctly (with validation)
4. Track which templates work best
5. Evolve templates based on usage patterns

**Specification requirements**:

### 1. Success Criteria (Measurable Outcomes)

Define what "successful tool" means:
- User can find appropriate template in under 30 seconds?
- Template selection accuracy >85%?
- Filled templates meet completeness criteria?
- Template usage tracked for improvement?

### 2. Core Features (WHAT the tool does)

Describe capabilities:
- Template storage and organization
- Template discovery (search, categorization)
- Template filling workflow
- Validation and completeness checks
- Usage analytics and recommendations

### 3. Constraints (Technical and Scope)

Define limitations:
- Platform (CLI? Web? IDE extension?)
- Storage format (JSON? Markdown? Database?)
- No implementation required (specification only)
- Must work with templates from Lessons 6-7

### 4. Non-Goals (Explicit Exclusions)

What this tool does NOT do:
- Not an AI chatbot (just template management)
- Not a prompt executor (doesn't call AI APIs)
- Not a prompt generator (uses human-written templates)

### 5. User Workflows (How people use it)

Describe key scenarios:
- New template creation
- Template selection for a task
- Template filling and validation
- Template refinement based on feedback

### 6. Validation Tests (Acceptance Criteria)

How to verify it works:
- Test: User searches "bug" → Sees Bug Fix Template
- Test: User fills incomplete template → Gets validation error
- Test: User uses template 5 times → Tool recommends it for similar tasks

---

## Specification Template (Use This Structure)

Here's the structure your specification should follow:

```markdown
# Prompt Template Library — Specification

**Version**: 1.0
**Author**: [Your Name]
**Date**: [Today's Date]

---

## 1. Overview

**WHAT**: [One-paragraph description of the tool]

**WHY**: [Problem this solves for developers]

**WHO**: [Target users and their context]

---

## 2. Success Criteria

The tool is successful when:
- [Measurable outcome 1]
- [Measurable outcome 2]
- [Measurable outcome 3]

---

## 3. Core Features

### Feature 1: [Name]
**Description**: [What it does]
**User value**: [Why it matters]

### Feature 2: [Name]
[...]

---

## 4. User Workflows

### Workflow 1: [Scenario Name]
**User goal**: [What user wants to accomplish]

**Steps**:
1. [Action 1]
2. [Action 2]
3. [Result]

**Success**: [How user knows it worked]

---

## 5. Technical Constraints

- Platform: [CLI/Web/IDE/Other]
- Storage: [File format, location]
- Dependencies: [External tools required]
- Performance: [Speed requirements]

---

## 6. Non-Goals

- [Feature explicitly out of scope]
- [Another exclusion]

---

## 7. Validation Tests

**Test 1**: [Scenario] → [Action] → [Expected Result]
**Test 2**: [Scenario] → [Action] → [Expected Result]

---

## 8. Open Questions

- [Decision not yet made]
- [Area needing clarification]
```

---

## Capstone Exercise Instructions

**Time budget**: 60 minutes

**Deliverable**: Specification document for Prompt Template Library

---

### Step 1: Define Success Criteria (10 minutes)

**Task**: Write 5-7 measurable outcomes that define "successful tool."

**Consider**:
- Speed (how fast can users find/fill templates?)
- Accuracy (does tool recommend right template?)
- Completeness (are filled templates high-quality?)
- Adoption (would team actually use this?)
- Learning (does tool improve over time?)

**Example success criteria**:

```markdown
## 2. Success Criteria

The Prompt Template Library is successful when:

1. **Template Discovery Speed**: User finds appropriate template in under 30 seconds (vs 5+ minutes writing custom prompt)

2. **Selection Accuracy**: Template recommendation matches user's task type >85% of the time (measured by user acceptance rate)

3. **Completeness**: Filled templates include all required sections >90% of the time (measured by validation pass rate)

4. **Adoption**: 80%+ of team uses library for recurring tasks within 2 weeks of onboarding

5. **Template Quality**: Templates evolved based on usage have 25%+ higher acceptance rate than v1.0

6. **Time Savings**: Average prompt creation time reduced from 8 minutes (custom) to 2 minutes (template)
```

**Your success criteria should be**:
- ✅ Measurable (numbers, percentages, time)
- ✅ User-focused (what users experience)
- ✅ Testable (you could verify these)

---

### Step 2: Define Core Features (15 minutes)

**Task**: Describe 5-7 capabilities the tool must have.

**Feature categories to consider**:
- Template storage and organization
- Template discovery (search, browse, recommend)
- Template filling and placeholder guidance
- Validation and completeness checking
- Usage tracking and analytics
- Template evolution (version control, refinement)

**Example feature specification**:

```markdown
## 3. Core Features

### Feature 1: Template Storage

**Description**: Store templates as structured markdown files with metadata (category, tags, version, usage count)

**User value**: Templates organized and versioned, easy to browse and maintain

**Structure**:
```
templates/
  bug-fix.md        (v2.1, category: diagnostic, tags: debugging, errors)
  refactoring.md    (v1.3, category: transformative, tags: code-quality)
  documentation.md  (v2.0, category: generative, tags: docs, api)
```

**Metadata format** (YAML frontmatter):
```yaml
---
name: Bug Fix Template
category: diagnostic
tags: [debugging, errors, troubleshooting]
version: 2.1
created: 2024-01-15
last_updated: 2024-03-10
usage_count: 47
avg_rating: 4.6
---
```

---

### Feature 2: Template Discovery

**Description**: CLI command to find templates by keyword, category, or task description

**User value**: Finds right template in seconds without browsing files

**Commands**:
```bash
# Search by keyword
$ prompt-library search "bug"
→ Shows: Bug Fix Template, Performance Bug Template

# Search by category
$ prompt-library list --category diagnostic
→ Shows all diagnostic templates

# Smart recommendation (AI-powered)
$ prompt-library recommend "authentication timeout issue"
→ Suggests: Bug Fix Template (95% match), Auth Template (87% match)
```

---

### Feature 3: Interactive Template Filling

**Description**: Guided prompt to fill each placeholder with validation

**User value**: Ensures no placeholders forgotten, provides guidance for each

**Workflow**:
```bash
$ prompt-library fill bug-fix

📝 Bug Fix Template (v2.1)
━━━━━━━━━━━━━━━━━━━━━━━━━━━

[COMPONENT]: authentication middleware
✓ Valid

[CURRENT_BEHAVIOR]: users logged out after 15 min
✓ Valid (descriptive)

[EXPECTED_BEHAVIOR]: stay logged in
⚠ Warning: Be more specific (e.g., "stay logged in for 24 hours")

[EXPECTED_BEHAVIOR]: stay logged in for 24 hours
✓ Valid

[ERROR_MESSAGE]:
❌ Required field. Enter error message or type "None"
[ERROR_MESSAGE]: None
✓ Valid

━━━━━━━━━━━━━━━━━━━━━━━━━━━
✓ Template complete (4/4 sections filled)

📋 Copy to clipboard? (Y/n)
```
```

**Your feature descriptions should include**:
- ✅ What the feature does (technical description)
- ✅ Why users need it (user value)
- ✅ How it works (examples, commands, workflows)

---

### Step 3: Define User Workflows (10 minutes)

**Task**: Describe 3-4 common scenarios showing how users interact with the tool.

**Workflow format**:
```markdown
### Workflow: [Scenario Name]

**User goal**: [What user wants to accomplish]

**Steps**:
1. User action → System response
2. User action → System response
3. Result

**Success**: [How user knows workflow succeeded]
```

**Example workflows to specify**:

**Workflow 1**: Creating a new template
**Workflow 2**: Finding and filling a template for a bug fix
**Workflow 3**: Refining a template based on usage feedback
**Workflow 4**: Browsing template categories

**Example**:

```markdown
### Workflow 1: Find and Fill Template for Bug Fix

**User goal**: Debug authentication timeout issue using appropriate template

**Steps**:
1. User runs: `prompt-library recommend "auth timeout bug"`
   → System suggests: Bug Fix Template (92% match)

2. User selects: Bug Fix Template
   → System starts interactive fill mode

3. User fills each placeholder with guidance
   → System validates completeness (warns if vague)

4. User confirms template complete
   → System copies to clipboard

5. User pastes into AI assistant (Claude Code, Cursor, etc.)
   → AI receives complete, well-structured debugging prompt

**Success**: User has complete debugging prompt in under 2 minutes (vs 8 minutes writing custom prompt)
```

---

### Step 4: Define Constraints and Non-Goals (5 minutes)

**Task**: Specify what the tool MUST work within (constraints) and what it will NOT do (non-goals).

**Example**:

```markdown
## 5. Technical Constraints

**Platform**: Command-line tool (Bash script or Python CLI)

**Storage**:
- Templates: Markdown files with YAML frontmatter
- Metadata: JSON file tracking usage stats
- Location: `~/.prompt-library/` directory

**Dependencies**:
- No external APIs (offline-first tool)
- No database (file-based storage)
- Clipboard access (for copy functionality)

**Performance**:
- Template search: under 1 second for 50 templates
- Interactive filling: Real-time validation (under 100ms per field)

---

## 6. Non-Goals

**Explicitly OUT of scope**:

1. **Not an AI chatbot**: Tool doesn't execute prompts or call AI APIs (just manages templates)

2. **Not a prompt generator**: Doesn't create new templates from scratch (uses human-written templates)

3. **Not IDE-integrated** (v1.0): CLI only, IDE extensions in v2.0

4. **Not collaborative** (v1.0): Single-user tool, team sharing in v2.0

5. **Not a prompt debugger**: Doesn't analyze prompt quality (just manages templates)
```

---

### Step 5: Define Validation Tests (10 minutes)

**Task**: Write 8-10 acceptance tests showing how to verify the tool works correctly.

**Test format**:
```markdown
**Test**: [Setup] → [Action] → [Expected Result]
```

**Example tests**:

```markdown
## 7. Validation Tests

### Template Discovery Tests

**Test 1**: Empty library → Run `search "bug"` → Returns "No templates found. Add templates to ~/.prompt-library/templates/"

**Test 2**: Library with Bug Fix Template → Run `search "debug"` → Returns Bug Fix Template

**Test 3**: Library with 3 templates → Run `list --category diagnostic` → Shows only diagnostic templates (filters correctly)

---

### Template Filling Tests

**Test 4**: Select Bug Fix Template → Fill all placeholders → Validation passes, template copied to clipboard

**Test 5**: Fill template, leave [ERROR_MESSAGE] blank → Validation fails with "Required field: ERROR_MESSAGE"

**Test 6**: Fill [CURRENT_BEHAVIOR] with vague "doesn't work" → Warning: "Be more specific (describe what actually happens)"

---

### Template Recommendation Tests

**Test 7**: Run `recommend "fix broken logout"` → Bug Fix Template ranked highest (>90% match)

**Test 8**: Run `recommend "explain how JWT works"` → No template match, suggests "Create custom prompt or create Explanation Template"

---

### Template Evolution Tests

**Test 9**: Use Bug Fix Template 10 times, rate 5 stars each time → Template marked "highly effective" in metadata

**Test 10**: Update Bug Fix Template v2.0 → v2.1 → Version history tracked, users notified of updates
```

---

### Step 6: Add Open Questions (5 minutes)

**Task**: List decisions you haven't made yet or areas needing clarification.

**Example**:

```markdown
## 8. Open Questions

**Q1: Template Recommendation Algorithm**
- Use keyword matching (simple, fast)?
- Use similarity scoring (TF-IDF, embeddings)?
- Use usage-based collaborative filtering (needs data)?
- **Decision needed**: Start with keyword matching (v1.0), add ML in v2.0?

**Q2: Template Validation Strictness**
- Block incomplete templates (strict)?
- Warn but allow incomplete (permissive)?
- **Decision needed**: Warn-only for v1.0 (don't block users), make configurable?

**Q3: Template Sharing**
- Export/import templates as `.zip` files?
- Git repository format (team can clone)?
- **Decision needed**: Git-based for v1.0 (developers familiar with git)?

**Q4: Usage Analytics Privacy**
- Track usage locally only (private)?
- Optional telemetry to improve recommendations (with consent)?
- **Decision needed**: Local-only for v1.0 (privacy-first), optional telemetry v2.0?
```

---

### Step 7: Peer Review Your Specification (5 minutes)

**Task**: Read your specification as if you're a developer implementing it. Ask:

**Completeness Check**:
- ✅ Could I implement this without asking clarification questions?
- ✅ Are success criteria measurable?
- ✅ Are features described with enough detail?
- ✅ Are workflows clear (step-by-step)?
- ✅ Are validation tests specific and actionable?

**Clarity Check**:
- ✅ Is the WHAT clear (vs HOW)?
- ✅ Are constraints explicit?
- ✅ Are non-goals preventing scope creep?

**If "no" to any**: Revise that section before finalizing.

---

## Framework Integration Checklist

Before finalizing your specification, verify it demonstrates understanding of **all four frameworks** taught in Chapter 12:

### Anthropic's Progressive Prompting
- ✅ QDD implements "ask before implementing" (Anthropic's step-by-step thinking)?
- ✅ Specification includes "simple → detailed → validated" progression?
- ✅ Tool guides users toward progressively refined prompts?

**Verification**: In your spec, look for language about validation gates, iterative refinement, or progressive complexity.

### Google's Structured Prompts
- ✅ Template library enforces structured sections (Command, Context, Constraints, Output)?
- ✅ Discovery system helps users find complete (not vague) templates?
- ✅ Validation checks that all structured elements present?

**Verification**: Check your Features and Validation Tests for completeness checking and structure enforcement.

### OpenAI's Constraint-Based Improvement
- ✅ Tool supports template evolution from v1 → v2 → v3 through constraint refinement?
- ✅ Validation tests show progression from basic → advanced templates?
- ✅ Non-goals explicitly exclude over-engineering (staying simple)?

**Verification**: Your Constraints section should mention version control and evolution.

### Zia's 8-Element Framework
- ✅ Specification templates encode all 8 elements (Command, Context, Logic, Roleplay, Formatting, Constraints, Examples, Questions)?
- ✅ Discovery system helps users find templates matching their task's logical structure?
- ✅ Templates guide complete reasoning (not just outputs)?

**Verification**: Your Feature descriptions should mention structured elements and reasoning guidance.

**Scoring**:
- All 4 frameworks explicitly addressed: 10/10 (capstone-quality)
- 3 frameworks clearly present: 7-8/10 (good spec)
- 2 or fewer frameworks: 5-6/10 (needs refinement)

---

## Specification Quality Rubric

Evaluate your specification against these criteria:

| Criterion | Poor (1) | Good (2) | Excellent (3) |
|-----------|----------|----------|---------------|
| **Success Criteria** | Vague ("user is happy") | Some measurable | All measurable with numbers |
| **Features** | Listed without detail | Described with examples | Includes commands, workflows |
| **Workflows** | Missing or incomplete | 2-3 workflows present | 4+ workflows, step-by-step |
| **Constraints** | Unspecified | Some technical details | Platform, storage, perf defined |
| **Validation Tests** | \<5 tests or vague | 5-7 specific tests | 8+ tests covering all features |
| **Clarity** | Ambiguous (needs Q&A) | Mostly clear | Implementable without questions |

**Target**: 15+ points (out of 18) = Specification-quality deliverable.

---

## What You've Learned

This capstone integrates all Chapter 12 concepts:

1. **Prompts as specifications** → You wrote a specification (not implementation)
2. **Specification-first** → You defined success criteria before describing features
3. **Iterative refinement** → You'll refine this spec based on peer review
4. **Success criteria** → You made outcomes measurable (30 sec search, 85% accuracy)
5. **Question-Driven Development** → Your open questions show gaps to address
6. **Templates** → You specified a tool that manages templates (meta!)
7. **Decision frameworks** → Your tool includes template selection logic

**Core achievement**: You've demonstrated prompt engineering as **specification skill**—the exact capability Jake Heller used to build a $650M product.

---

## Next Steps After This Chapter

**You've completed Chapter 12: Prompt Engineering for AI-Driven Development.**

You now understand:
- ✅ Prompts as specifications (WHAT, not HOW)
- ✅ Prompt structure (Intent → Constraints → Success Criteria)
- ✅ Iterative refinement (60% → 97% through iteration)
- ✅ Specification-first prompting
- ✅ Question-Driven Development
- ✅ Reusable templates for recurring tasks
- ✅ Template selection decision frameworks
- ✅ Writing peer-reviewable specifications

**Coming in Chapter 16**: Context Engineering for AI-Driven Development
- Context windows and token limits
- Progressive loading strategies
- Context compression techniques
- Memory files (CLAUDE.md, architecture.md)
- Tool selection based on context capacity

**Your Capstone Artifact**: Keep your Prompt Template Library specification. In Part 4 (Chapters 12+), when you learn Python, you'll be able to **implement** this specification—turning your design into working code.

---

## Layer 3 Intelligence: Using the Prompt Template Designer Skill

You've learned to USE templates (Layer 2). Now learn to CREATE reusable templates as intelligence design (Layer 3).

The `@prompt-template-designer` skill encodes the template creation process using the **Persona + Questions + Principles (P+Q+P)** pattern you've practiced throughout this chapter.

### When to Create a Template

Use the skill's **6 Analysis Questions** to decide if a prompt should become a template:

**Question 1: Recurrence Check** — "Have I used this prompt pattern 2+ times?"
- **If YES**: Pattern recurs, create template
- **If NO**: Keep as one-off prompt

**Question 2: Variation Analysis** — "What stays constant vs what changes?"
- **Constants**: Template structure (Intent format, core constraints, validation criteria)
- **Variants**: Parameters (file names, error messages, project context)

**Question 3: Complexity Check** — "Does this prompt have 5+ decision points?"
- Count decisions: action verb, constraints, format, validation, edge cases, thresholds, context
- **5+ decisions** = High template value (reduces cognitive load)
- **1-3 decisions** = Low template value (just write prompt directly)

**Question 4: Domain Knowledge Check** — "Does this encode project-specific intelligence?"
- High-value templates capture: team conventions, quality standards, architecture patterns, best practices
- Low-value templates are generic: "Ask AI a question", "Generate code"

**Question 5: Parameter Design** — "What parameters make this flexible but not vague?"
- Use descriptive names: `{{TARGET_FILE}}` not `{{X}}`
- Add type hints: `{{ERROR_MESSAGE}}` (Type: text)
- Provide examples: `Example: Permission denied`

**Question 6: Template Organization** — "How should I organize for discoverability?"
- By development phase (requirements, implementation, testing, docs)
- By action verb (create, debug, refactor, optimize)
- By domain (backend-api, frontend-components, devops)

### Applying the Skill to Your Capstone

For your Prompt Template Library specification, use the skill to design the template creation workflow:

**Workflow: Creating a New Template**
1. User writes successful prompt 2+ times → Recognizes pattern
2. System asks 6 analysis questions → Evaluates template worthiness
3. User identifies constants vs variants → Extracts structure
4. System generates template with parameters → Adds metadata
5. User provides example → Validates template
6. System stores template → Enables future reuse

**This workflow IS the prompt-template-designer skill operationalized.**

### Practice: Design a Template Using the Skill

**Your task**: You've written Git commit messages 5+ times using Conventional Commits format. Create a reusable template.

**Apply the 6 questions**:
1. Recurrence: 5 uses → YES, create template
2. Variation:
   - **Constants**: Conventional Commits format, imperative mood, &lt;50 char limit
   - **Variants**: `CHANGES_MADE`, `TICKET_ID`, `SCOPE`, `TYPE`
3. Complexity: 7 decisions (type, scope, ticket, subject length, body format, business value, imperative mood) → High value
4. Domain knowledge: Team's Jira convention, business value focus → Project-specific
5. Parameters: Descriptive (`CHANGES_MADE` with examples), typed, required vs optional marked
6. Organization: Store in `templates/generate/git-commit.md`

**Result**: Here's the complete template you would create:

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

## Parameters

### CHANGES_MADE
- **Type**: composite (list of changes with context)
- **Description**: List of changes made in this commit with technical details
- **Example**:
  - Added JWT refresh endpoint (/auth/refresh)
  - Extended token expiration from 1h to 24h
  - Fixed logout race condition in session handler
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

GENERATE Git commit message using Conventional Commits format

CHANGES MADE:
[Insert your changes here]

CONSTRAINTS:
- Format: [TYPE]([SCOPE]): <description> [[JIRA_TICKET]]
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

## Example (Filled)

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
```

**Key insight**: The skill teaches you to think systematically about template design, not just copy existing templates. This is Layer 3 intelligence—creating reusable patterns.

---

## Try With AI

Ready to apply all 8 prompt engineering elements to create a specification-quality capstone project?

**🔍 Explore Specification-First Thinking:**
> "Show me the difference between implementation-first and specification-first. For a 'Prompt Template Library tool': (1) start coding immediately with vague requirements, (2) write a complete specification first (success criteria, features, workflows, constraints, validation tests). Compare the clarity and implementability of both approaches."

**🎯 Practice QDD for Specification Refinement:**
> "I've drafted a Prompt Template Library specification [paste your draft]. Before suggesting improvements, ask me 8 clarifying questions about: success criteria measurability, missing features, workflow ambiguities, validation test gaps, and under-specified constraints. Based on my answers, identify specification improvements."

**🧪 Test Specification Completeness:**
> "Review my specification using these criteria: Could a developer implement without questions? Are success criteria measurable? Are features detailed with examples? Are workflows step-by-step? Are validation tests specific? For each criterion, score 1-3 and explain gaps."

**🚀 Apply to Your Capstone:**
> "Help me create a complete specification for [your capstone project]. I'll describe my idea. You guide me through: (1) defining measurable success criteria, (2) listing core features with user value, (3) detailing user workflows, (4) specifying constraints, (5) writing validation tests. Produce a peer-reviewable specification."

---
