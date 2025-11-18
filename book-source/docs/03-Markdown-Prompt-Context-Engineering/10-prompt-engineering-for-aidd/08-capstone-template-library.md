---
title: "Capstone: Prompt Template Library Specification"
chapter: 10
lesson: 8
part: 3
duration_minutes: 60

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Specification-Driven Prompt Engineering"
    proficiency_level: "B1"
    category: "Spec-Driven Integration"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student writes complete specification for Prompt Template Library tool"

learning_objectives:
  - objective: "Create complete specification for Prompt Template Library tool using Chapter 10 principles"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Peer-reviewable specification document with success criteria and validation tests"

cognitive_load:
  new_concepts: 5
  assessment: "5 concepts (specification integration, template library architecture, version control, template discovery, usage analytics) at B1 threshold ✓"
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
- User can find appropriate template in \<30 seconds?
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

1. **Template Discovery Speed**: User finds appropriate template in \<30 seconds (vs 5+ minutes writing custom prompt)

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

**Success**: User has complete debugging prompt in \<2 minutes (vs 8 minutes writing custom prompt)
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
- Template search: \<1 second for 50 templates
- Interactive filling: Real-time validation (\<100ms per field)

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

This capstone integrates all Chapter 10 concepts:

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

**You've completed Chapter 10: Prompt Engineering for AI-Driven Development.**

You now understand:
- ✅ Prompts as specifications (WHAT, not HOW)
- ✅ Prompt structure (Intent → Constraints → Success Criteria)
- ✅ Iterative refinement (60% → 97% through iteration)
- ✅ Specification-first prompting
- ✅ Question-Driven Development
- ✅ Reusable templates for recurring tasks
- ✅ Template selection decision frameworks
- ✅ Writing peer-reviewable specifications

**Coming in Chapter 11**: Context Engineering for AI-Driven Development
- Context windows and token limits
- Progressive loading strategies
- Context compression techniques
- Memory files (CLAUDE.md, architecture.md)
- Tool selection based on context capacity

**Your Capstone Artifact**: Keep your Prompt Template Library specification. In Part 4 (Chapters 12+), when you learn Python, you'll be able to **implement** this specification—turning your design into working code.

---

## Try With AI (Optional Extension)

**Challenge**: Use Question-Driven Development to refine your specification.

**Part 1: Prompt AI to Review Your Spec**

```markdown
I've written a specification for a Prompt Template Library tool (see below).

Before suggesting improvements, ask me 5-7 clarifying questions about:
- Success criteria measurability
- Feature completeness (missing capabilities?)
- Workflow clarity (ambiguous steps?)
- Validation test coverage (gaps in testing?)
- Constraints (under-specified technical details?)

Based on my answers, suggest specification improvements.

[Paste your specification]
```

**Part 2: Answer AI's Questions**

AI will identify gaps (e.g., "How should template versioning work? How are conflicts resolved?").

Answer each question specifically.

**Part 3: Refine Specification to v1.1**

Based on AI feedback, update your specification:
- Add missing features
- Clarify vague sections
- Add validation tests for edge cases
- Specify open questions as decisions

**Part 4: Validate Refinement**

Re-run completeness check:
- Could a developer implement this without questions? (If yes → specification-quality ✓)

**Key learning**: Specifications improve through iteration—just like prompts improve from 60% to 97% through refinement.

Your v1.1 specification is now a portfolio-quality artifact demonstrating your prompt engineering mastery.
