---
title: "Template Selection and Decision Frameworks"
chapter: 10
lesson: 7
part: 3
duration_minutes: 35

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Template Selection Decision Framework"
    proficiency_level: "B1"
    category: "Intelligence Design"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student evaluates tasks to select appropriate template or custom prompt approach"

learning_objectives:
  - objective: "Apply decision criteria to select appropriate prompt template or custom approach"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Template selection decisions for 5 development scenarios"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (decision frameworks, template applicability, task classification, hybrid approaches, template adaptation, custom vs template tradeoffs, selection criteria) at B1 threshold ✓"
---

# Template Selection and Decision Frameworks

You now have prompt templates for common tasks (bug fixes, refactoring, documentation). But here's the question that determines whether templates actually save you time:

**"Should I use a template for THIS task, or write a custom prompt?"**

Use a template when it doesn't fit → waste time fighting the template.
Write custom prompts when a template exists → waste time reinventing the pattern.

This lesson teaches you **decision frameworks** for template selection: systematic questions to ask that reveal whether a task is template-appropriate or requires custom prompting.

By the end of this lesson, you'll evaluate any development task and confidently choose the right approach in under 30 seconds.

---

## The Template Selection Problem

You have a task: "Explain how the authentication middleware works."

You have templates:
- Bug Fix Template
- Refactoring Template
- Documentation Template

**Questions**:
1. Which template fits?
2. Or should you write a custom prompt?
3. How do you decide quickly?

**Common mistakes**:
- ❌ Force task into wrong template ("It's kinda like documentation...")
- ❌ Spend 5 minutes deciding (negates template time-saving benefit)
- ❌ Always use custom prompts (ignore templates you've built)

**The fix**: A decision framework that takes 20-30 seconds to apply.

---

## Template Selection Framework

Ask these three questions **in order**:

### Question 1: What is the task category?

**Categories**:
- **Diagnostic** — Finding/fixing problems (bugs, errors, performance issues)
- **Transformative** — Changing existing code/docs (refactoring, optimization, migration)
- **Generative** — Creating new artifacts (documentation, tests, features, configs)
- **Explanatory** — Understanding or teaching concepts (code explanation, onboarding)
- **Evaluative** — Assessing quality (code review, security audit, architecture review)

**Template mapping**:
| Category | Likely Template |
|----------|-----------------|
| Diagnostic → Bug Fix Template |
| Transformative → Refactoring Template |
| Generative → Documentation/Feature Template |
| Explanatory → Custom (or Explanation Template if you've created one) |
| Evaluative → Code Review Template (or custom) |

**Example**:
- Task: "Fix login timeout bug" → **Diagnostic** → Bug Fix Template ✓
- Task: "Simplify this function" → **Transformative** → Refactoring Template ✓
- Task: "Explain how JWT works" → **Explanatory** → Custom prompt (educational, not task-execution)

---

### Question 2: Does this task match template assumptions?

Every template has **built-in assumptions**:

**Bug Fix Template assumes**:
- Problem exists in EXISTING code (not design phase)
- You can describe current vs expected behavior
- There's reproducibility info to provide

**Refactoring Template assumes**:
- Code WORKS but needs improvement
- You can specify what stays same vs what can change
- Goal is quality improvement (not bug fix, not new feature)

**Documentation Template assumes**:
- Subject matter EXISTS (API, feature, code to document)
- You know the audience and their knowledge level
- Output format is known (markdown, JSDoc, OpenAPI, etc.)

**Decision**:
- If task matches ALL assumptions → Use template ✓
- If task violates 1+ assumptions → Custom prompt or adapt template

**Examples**:

**Task**: "Debug authentication bug in code I'm about to write"
- ❌ Bug Fix Template (assumes existing code, but code doesn't exist yet)
- ✅ Custom prompt (this is design-phase problem prevention, not debugging)

**Task**: "Refactor this working function to add new feature"
- ❌ Refactoring Template (assumes behavior stays same, but you're adding behavior)
- ✅ Custom prompt or Feature Template (this is enhancement, not refactoring)

**Task**: "Document our future API design"
- ❌ Documentation Template (assumes API exists, but it's still being designed)
- ✅ Custom prompt or Specification Template (this is spec writing, not documentation)

---

### Question 3: Is customization effort less than template savings?

Templates save time by providing structure. But if you spend 10 minutes adapting a template to fit a unique task, you've wasted time.

**Decision rule**:

```
IF (time to fill template + time to adapt template) < (time to write custom prompt)
  THEN use template
ELSE
  write custom prompt
```

**Estimation heuristic**:

| Scenario | Template Time | Custom Time | Decision |
|----------|--------------|-------------|----------|
| Straightforward bug fix | 2 min fill | 5 min custom | Template ✓ |
| Standard refactoring | 3 min fill | 7 min custom | Template ✓ |
| Unique architecture task | 8 min adapt | 6 min custom | Custom ✓ |
| Hybrid (refactor + feature) | 6 min adapt | 5 min custom | Custom ✓ |

**Example**:

**Task**: "Refactor AND add retry logic to API client"

**Template approach**:
- Refactoring Template doesn't fit (behavior changes with retry logic)
- Spend 5-6 minutes writing custom sections to handle hybrid task

**Custom prompt approach**:
- Write prompt directly: "Refactor API client to improve readability AND add retry logic with exponential backoff. Constraints: preserve existing API, add tests for retry behavior."
- Takes 3-4 minutes

**Decision**: Custom prompt (faster than adapting template for hybrid task).

---

## Template Selection Decision Tree

Use this flowchart for fast decisions:

```
Start: I have a development task

┌─────────────────────────────────────┐
│ Q1: Is this task recurring?         │
│ (Have I done similar task 2+ times?)│
└──────────┬──────────────────────────┘
           │
    ┌──────┴──────┐
    │             │
   NO            YES
    │             │
    v             v
[Custom]   ┌──────────────────────────┐
           │ Q2: Does existing         │
           │ template match category?  │
           └──────┬───────────────────┘
                  │
           ┌──────┴──────┐
           │             │
          NO            YES
           │             │
           v             v
      [Custom]    ┌─────────────────────┐
                  │ Q3: Does task match  │
                  │ template assumptions?│
                  └──────┬──────────────┘
                         │
                  ┌──────┴──────┐
                  │             │
                 NO            YES
                  │             │
                  v             v
         ┌────────────┐   [Use Template]
         │ Q4: Adapt  │
         │ template   │
         │ faster than│
         │ custom?    │
         └──────┬─────┘
                │
         ┌──────┴──────┐
         │             │
        YES           NO
         │             │
         v             v
   [Adapt          [Custom]
    Template]
```

**Decision time**: 20-30 seconds to walk this tree.

---

## Decision Framework in Action

Let's apply the framework to five realistic scenarios.

### Scenario 1: "Fix the broken logout button"

**Q1: Recurring?** Yes (bug fixes are common)
**Q2: Template exists?** Yes (Bug Fix Template)
**Q3: Matches assumptions?** Yes (existing code, problem described, reproducible)

**Decision**: ✅ Bug Fix Template

**Prompt**:
```markdown
TASK: Debug logout button issue

PROBLEM:
Current: Logout button doesn't respond to clicks
Expected: User logged out, redirected to login page
Error: None (no console errors, button just doesn't respond)

REPRODUCIBILITY:
- Steps: Login → Click logout → Nothing happens
- Frequency: Always
- Environment: Production only (works in dev)

CONTEXT:
- Last working: Version 1.4.2
- Recent changes: Updated React from 17 to 18 (2024-03-10)
- Impact: All users affected

[... rest of template]
```

---

### Scenario 2: "Explain how our caching layer works for new team member"

**Q1: Recurring?** Yes (onboarding explanations)
**Q2: Template exists?** No (no Explanation Template created yet)
**Q3: Custom or create template?** If explaining architecture to new hires is common (3+ times/year), create template. Otherwise, custom.

**Decision**: ✅ Custom prompt (or create Explanation Template if recurring)

**Custom Prompt**:
```markdown
Explain our caching layer architecture to a new backend developer.

AUDIENCE: Backend developer, familiar with Redis, first week on team

WHAT TO EXPLAIN:
1. Cache structure (what's cached, key naming conventions)
2. Cache invalidation strategy (when/how cache is cleared)
3. Common patterns team uses (read-through, write-through, cache-aside)

FORMAT:
- High-level overview (2-3 paragraphs)
- Then specific examples from our codebase
- Include file paths for reference

SUCCESS CRITERIA:
Developer understands where to look when debugging cache issues
```

---

### Scenario 3: "Refactor user service AND add email validation"

**Q1: Recurring?** Refactoring is, but hybrid tasks vary
**Q2: Template exists?** Refactoring Template exists but doesn't fit (behavior changes with email validation)
**Q3: Adapt template?** Would need to heavily modify constraints section (5+ min adapt)
**Q4: Faster custom?** Yes (3 min to write from scratch)

**Decision**: ✅ Custom prompt

**Custom Prompt**:
```markdown
Refactor UserService class for readability AND add email validation feature.

REFACTORING GOALS:
- Reduce method complexity (currently 3 methods >20 lines)
- Extract validation logic into separate validator class

NEW FEATURE:
- Email validation: check format, domain MX records, disposable email blocking
- Throw ValidationError with specific message on invalid email

CONSTRAINTS:
- API changes allowed (this is v2.0 breaking change)
- Must maintain existing tests (57 passing)
- Email validation must be reusable (other services will need it)

OUTPUT:
1. Refactored UserService
2. New EmailValidator class
3. Updated tests
4. Migration guide (breaking changes)
```

---

### Scenario 4: "Document the /api/users endpoints"

**Q1: Recurring?** Yes (API documentation)
**Q2: Template exists?** Yes (Documentation Template)
**Q3: Matches assumptions?** Yes (API exists, audience is developers, format is markdown)

**Decision**: ✅ Documentation Template

**Filled Template**:
```markdown
TASK: Create documentation for User Management API

AUDIENCE:
- Primary: Backend developers integrating our user service
- Knowledge: Familiar with REST, unfamiliar with our system
- Use case: First-time integration

SCOPE:
- Document: GET /users, GET /users/:id, POST /users, PATCH /users/:id, DELETE /users/:id
- Exclude: Authentication endpoints (separate guide)

FORMAT:
- Markdown
- curl examples
- Quick reference (under 400 words)

REQUIRED SECTIONS:
1. Endpoint list with one-line descriptions
2. Request/response schemas for each
3. Example requests (curl)
4. Common error codes

SUCCESS CRITERIA:
Developer can integrate user CRUD without asking questions
```

---

### Scenario 5: "Optimize database query performance"

**Q1: Recurring?** Yes (performance optimization)
**Q2: Template exists?** Possibly Refactoring Template (optimization is refactoring for performance)
**Q3: Matches assumptions?** Partially (code works but slow, goal is improvement)

**Decision**: ✅ Adapt Refactoring Template OR create Performance Template if recurring

**Adapted Refactoring Template**:
```markdown
TASK: Optimize getUserOrders database query

CURRENT ISSUES:
- Problem: Query takes 3.2 seconds with 1000+ orders (timeout risk)
- Impact: User dashboard page load unacceptably slow

OPTIMIZATION GOALS:
Primary goal: Performance
Success criteria: Query completes in \<500ms for 10,000 orders

WHAT MUST STAY THE SAME:
- Query results: Identical data returned (no behavior change)
- Sort order: Orders sorted newest-first
- Filters: Support same filtering options

WHAT CAN CHANGE:
- Database indexes: Can add/modify
- Query structure: Can use joins, subqueries, CTEs
- Pagination: Can add limit/offset (breaking change acceptable)

CONSTRAINTS:
- Database: PostgreSQL 14
- No schema changes to orders table (used by other services)
- Must work with existing 50M order records

REQUESTED OUTPUT:
1. Performance analysis (EXPLAIN ANALYZE before/after)
2. Optimized query with explanation
3. Index recommendations
4. Migration plan for index creation
```

---

## When to Create a NEW Template

You'll encounter tasks that don't fit existing templates but ARE recurring. This signals: **time to create a new template**.

**New Template Criteria**:

1. **Task recurs 3+ times** (bug fix, refactoring, docs have recurred 10+ times, so templates exist)
2. **Pattern is consistent** (same structure, same questions to ask)
3. **Team would benefit** (not just you, but 2+ people perform this task type)

**Examples of template-worthy new patterns**:

✅ **Git Commit Message Template** (recurring every commit, consistent format)
✅ **Code Review Template** (recurring every PR, consistent checklist)
✅ **Performance Optimization Template** (recurring when scaling, consistent analysis needed)
✅ **Security Audit Template** (recurring for sensitive features, consistent security checks)
✅ **API Specification Template** (recurring for each endpoint design, consistent structure)

**Process**:
1. Perform task custom 2-3 times
2. Notice pattern in your prompts
3. Extract common structure
4. Create template v1.0
5. Use and refine to v2.0
6. Share with team

---

## Hybrid Approach: Template + Custom Sections

Sometimes a template is 80% right but needs customization. Use the **hybrid approach**:

**Structure**:
```markdown
[Use template sections that fit]

--- CUSTOM SECTION ---
[Add task-specific requirements]

--- CONTINUE TEMPLATE ---
[Use remaining template sections]
```

**Example**: Documentation + Tutorial

```markdown
[Documentation Template sections 1-3]

--- CUSTOM ADDITION ---
TUTORIAL SECTION:
Include step-by-step tutorial showing:
1. Installation
2. Basic usage example
3. Common customization

--- CONTINUE TEMPLATE ---
[Documentation Template sections 4-5]
```

**When to use hybrid**:
- Template covers 70%+ of task
- Custom additions are 1-2 sections
- Faster than full custom prompt

**When NOT to use hybrid**:
- Template covers \<50% (write custom instead)
- Customization requires rewriting multiple sections (write custom instead)

---

## Template Selection Mistakes

### Mistake 1: Template Forcing

❌ **Wrong**: "I have a Documentation Template, so I'll force this architecture explanation into it."

*Architecture explanation needs different structure than API docs (concepts vs endpoints).*

✅ **Right**: "Documentation Template is for APIs. Architecture explanation needs custom prompt or Architecture Template."

---

### Mistake 2: Always Custom (Ignoring Templates)

❌ **Wrong**: "I'll just write prompts from scratch every time."

*Wastes 5 minutes on recurring tasks, forgets critical sections, inconsistent results.*

✅ **Right**: "Check if template exists and fits. If yes, use it. If no, write custom."

---

### Mistake 3: No Template Evolution

❌ **Wrong**: "Template doesn't quite fit, I'll write custom."

*Template stays stagnant, never improves to cover common variations.*

✅ **Right**: "Template almost fits. I'll adapt it now, then update template to v2 to cover this variation."

**Template evolution example**:

**Bug Fix Template v1**: Only handles code bugs
**Usage reveals**: Used for deployment issues, configuration bugs too
**Bug Fix Template v2**: Adds "Environment configuration" section
**Result**: Template now covers 90% of debugging tasks (vs 70% in v1)

---

## Decision Framework Summary

**Fast template selection (30 seconds)**:

1. **Recurring?** → No: Custom | Yes: Next question
2. **Template exists?** → No: Custom (or create template) | Yes: Next question
3. **Assumptions match?** → No: Custom or adapt | Yes: Use template ✓
4. **Adaptation faster?** → No: Custom | Yes: Adapt template ✓

**Template categories**:
- Diagnostic → Bug Fix
- Transformative → Refactoring / Optimization
- Generative → Documentation / Feature / Config
- Explanatory → Custom or Explanation Template
- Evaluative → Code Review / Audit Template

**Template evolution**:
- Create from 2-3 examples
- Use 5+ times
- Refine based on gaps
- Share with team

---

## What You've Learned

You've learned systematic decision frameworks for choosing templates vs custom prompts:

1. **Selection framework** — 3-question decision tree (recurring? template exists? assumptions match?)
2. **Template categories** — 5 task types and their template mapping
3. **Hybrid approach** — Using templates with custom additions when 80% fit
4. **Template evolution** — Updating templates based on usage patterns
5. **Common mistakes** — Template forcing, always custom, no evolution

**Core principle**: Templates save time on recurring, structured tasks. Custom prompts handle unique or complex tasks. Decision frameworks help you choose correctly in 30 seconds.

In the final lesson (Lesson 8), you'll apply everything you've learned to create a **Prompt Template Library Specification**—a capstone project that orchestrates prompt engineering principles into a reusable tool for your entire team.

---

## Try With AI

**Your Task**: Evaluate 5 development tasks and select template vs custom approach.

For each task below:
1. Identify task category (Diagnostic, Transformative, Generative, Explanatory, Evaluative)
2. Determine if template exists and fits
3. Decide: Template / Custom / Adapt
4. Write rationale (1-2 sentences)

---

**Task A**: "Explain the difference between authentication and authorization to a junior developer"

**Your decision**:
- Category: _______
- Template or Custom: _______
- Rationale: _______

---

**Task B**: "Fix the 500 error when users upload files larger than 10MB"

**Your decision**:
- Category: _______
- Template or Custom: _______
- Rationale: _______

---

**Task C**: "Create OpenAPI documentation for our new payment API endpoints"

**Your decision**:
- Category: _______
- Template or Custom: _______
- Rationale: _______

---

**Task D**: "Refactor the authentication module to support OAuth in addition to current JWT implementation"

**Your decision**:
- Category: _______
- Template or Custom: _______
- Rationale: _______

---

**Task E**: "Review this pull request for security vulnerabilities and code quality"

**Your decision**:
- Category: _______
- Template or Custom: _______
- Rationale: _______

---

**Answers** (Compare yours):

**Task A**:
- Category: **Explanatory**
- Decision: **Custom** (or Explanation Template if you've created one)
- Rationale: Educational explanation, not task execution. Needs concept comparison, not structured problem-solving.

**Task B**:
- Category: **Diagnostic**
- Decision: **Bug Fix Template ✓**
- Rationale: Classic bug (error message, reproducible, existing code). Template fits perfectly.

**Task C**:
- Category: **Generative**
- Decision: **Documentation Template ✓** (possibly adapt for OpenAPI format)
- Rationale: API documentation, known audience (developers), structured format. Template sections apply.

**Task D**:
- Category: **Transformative** (hybrid: refactoring + feature addition)
- Decision: **Custom** (doesn't fit pure Refactoring Template)
- Rationale: Behavior changes (adds OAuth), not just quality improvement. Too complex for standard template.

**Task E**:
- Category: **Evaluative**
- Decision: **Code Review Template** (if exists) or **Custom**
- Rationale: Structured checklist task (security, quality). If you've created review template, use it. Otherwise, custom with security/quality criteria.

---

**Reflection Questions**:

1. How long did each decision take? (Goal: \<30 seconds per task)
2. Did you consider template assumptions or just category? (Assumptions matter!)
3. For tasks where you chose "custom," would this task recur enough to justify creating a template?

**Next time you face a real development task**: Run through this 3-question framework. Practice makes template selection automatic.
