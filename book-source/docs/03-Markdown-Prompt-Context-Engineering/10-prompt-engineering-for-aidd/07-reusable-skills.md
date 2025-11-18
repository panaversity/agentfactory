# Lesson 7: Creating Reusable Prompt Skills for Documentation and Generation

---

## What Should This Lesson Enable You to Do?

By the end of this lesson, you should be able to:

- **Design reusable prompt skills** using Persona + Questions + Principles pattern
- **Test skills for reusability** across 3+ domains to validate domain-agnostic patterns
- **Understand context chunking** for large documentation exploration
- **Create decision frameworks** that guide AI reasoning (not rigid instructions)
- **Validate skill quality** through cross-domain testing and transfer assessment

**Success criteria**: You can create two reusable skills (documentation-exploration and markdown-generation), test them across multiple domains (FastAPI, Django, Flask docs), and explain why these skills will transfer to Python coding tasks in Part 4.

---

## The Reusable Intelligence Problem

### The Scenario (Multiple Perspectives)

**Scenario A: Software Architect**
You are a **Software Architect** evaluating three Python web frameworks (FastAPI, Django, Flask) for a new project. You need to understand core design philosophies, architectural patterns, and trade-offs.

**Scenario B: Educator/Content Creator**
You are an **Educator** creating a tutorial series comparing web frameworks. You need to extract key concepts, identify learning progressions, and explain design decisions to beginners in clear, consistent ways across multiple frameworks.

**Scenario C: Student/Learner**
You are a **Student** exploring Python libraries for your capstone project. You need to understand which library fits your needs, what the learning curve looks like, and how it compares to alternatives you've heard about.

**Common problem across all roles**: Every time you explore new documentation (Django, Flask, PostgreSQL, React), you're reinventing the same exploration process. You ask similar questions, apply similar reasoning, but start from scratch each time.

**Insight**: The pattern for "exploring technical documentation conceptually" recurs across ALL frameworks AND all roles. Why not encode it once?

**Solution**: Create a **reusable skill** that applies to ANY framework documentation, regardless of your role.

**This lesson teaches you how to design, test, and validate such skills.**

---

## Concept 1: The Persona + Questions + Principles Pattern

### Why This Pattern Works

Reusable skills need three components:

1. **Persona**: The cognitive stance (HOW to think about the problem)
2. **Questions**: The analytical frameworks (WHAT to ask)
3. **Principles**: The decision frameworks (HOW to decide)

**This pattern activates reasoning mode** instead of prediction mode. Instead of AI sampling generic responses, AI applies systematic frameworks.

---

### Component 1: Persona (Cognitive Stance)

**Purpose**: Define HOW to think about the task.

**Bad persona** (too vague):
> "Think like an expert"

**Good persona** (specific cognitive stance):
> "Think like a technical researcher extracting architectural patterns and design decisions from documentation, not just feature lists"

**What this does**:
- Focuses AI on architecture (not features)
- Prioritizes design decisions (WHY) over usage instructions (HOW)
- Extracts patterns (transferable insights) over specifics (framework details)

---

### Component 2: Questions (Analytical Framework)

**Purpose**: Define WHAT analytical questions guide exploration.

**For documentation-exploration skill, 5 questions**:

1. **What problem does this framework/library solve?** (Purpose, not features)
2. **What design decisions differentiate it from alternatives?** (Comparative insight)
3. **What are the core abstractions?** (Mental model building)
4. **What constraints or limitations exist?** (Trade-offs, not just benefits)
5. **How would I evaluate if this fits my use case?** (Decision framework)

**These questions are domain-agnostic**: They apply to FastAPI, Django, Flask, React, PostgreSQL, or ANY framework documentation.

---

### Component 3: Principles (Decision Framework)

**Purpose**: Define HOW to make decisions during exploration.

**For documentation-exploration skill, 5 principles**:

1. **Conceptual before implementation**: Understand WHY before HOW
2. **Decision frameworks over absolutes**: "When to use X" not "X is best"
3. **Falsifiable understanding**: Can you explain trade-offs?
4. **Progressive layers**: Conceptual → Logical → Physical → Operational
5. **Context chunking**: For large docs, explore sections sequentially, not all at once

**These are frameworks, not rules**: They guide judgment, not replace it.

---

## Concept 2: Designing the documentation-exploration Skill

### Step 1: Recognize the Recurring Pattern

**You've explored FastAPI documentation** (Lesson 4). Now you're exploring Django and Flask.

**Ask**: What's the SAME across all three explorations?

**Same patterns**:
- You want design rationale (not just features)
- You want comparison to alternatives (not just "this is good")
- You want mental models (abstractions, patterns)
- You want trade-offs (limitations, constraints)

**Different details**:
- FastAPI: Async, type hints, dependency injection
- Django: ORM, batteries-included, MTV pattern
- Flask: Minimal, unopinionated, extensions

**Insight**: The exploration PROCESS is reusable. The framework-specific CONTENT is not.

---

### Step 2: Extract the Reusable Pattern

**Write the skill**:

```markdown
# documentation-exploration

## Persona
"Think like a technical researcher extracting architectural patterns and design
decisions from documentation, not just feature lists"

## Questions
1. What problem does this framework/library solve?
2. What design decisions differentiate it from alternatives?
3. What are the core abstractions?
4. What constraints or limitations exist?
5. How would I evaluate if this fits my use case?

## Principles
1. Conceptual before implementation
2. Decision frameworks over absolutes
3. Falsifiable understanding
4. Progressive layers (conceptual → logical → physical → operational)
5. Context chunking for large docs
```

---

### Step 3: Test Across Domains

**Domain Test 1: FastAPI Documentation**

**Apply skill**:
- Q1: FastAPI solves async web API development with type safety
- Q2: Design decision: Type hints for request/response validation (different from Flask)
- Q3: Core abstractions: Pydantic models, dependency injection, path operations
- Q4: Constraints: Requires Python 3.7+, learning curve for async patterns
- Q5: Fits if: You need high-performance async APIs with strong typing

**Result**: ✅ Skill provided systematic framework understanding

---

**Domain Test 2: Django Documentation**

**Apply skill**:
- Q1: Django solves full-stack web development with batteries included
- Q2: Design decision: ORM-first (different from SQLAlchemy raw SQL)
- Q3: Core abstractions: Models, Views, Templates (MTV), Admin interface
- Q4: Constraints: Opinionated structure, heavier than Flask, monolithic
- Q5: Fits if: You want rapid development with built-in admin/auth

**Result**: ✅ Skill provided systematic framework understanding

---

**Domain Test 3: Flask Documentation**

**Apply skill**:
- Q1: Flask solves lightweight web development with flexibility
- Q2: Design decision: Minimal core, extensions for features (different from Django)
- Q3: Core abstractions: Routes, Views, Blueprints, Context
- Q4: Constraints: Need to add features manually, less structure than Django
- Q5: Fits if: You want control over architecture decisions

**Result**: ✅ Skill provided systematic framework understanding

---

### Reusability Validation

**The documentation-exploration skill worked across**:
- FastAPI (async-focused framework)
- Django (full-stack framework)
- Flask (minimalist framework)

**Same 5 questions, same process, different insights per framework.**

**Conclusion**: This skill is **domain-agnostic** and will transfer to:
- Python library docs (Pandas, NumPy, SQLAlchemy)
- JavaScript frameworks (React, Vue, Angular)
- Database docs (PostgreSQL, MongoDB, Redis)
- System tools (Docker, Kubernetes, Terraform)

---

## Concept 3: Context Chunking for Large Documentation

### The Problem with Large Docs

Framework documentation often has 50-100+ pages. You can't read everything linearly.

**Without context chunking**, you either:
- Read too broadly (surface-level understanding)
- Read too deeply (lost in details, miss big picture)

**With context chunking**, you explore in layers:
1. **Layer 1: Overview** (What is this? Why does it exist?)
2. **Layer 2: Core Concepts** (What are the key abstractions?)
3. **Layer 3: Comparisons** (How does this differ from alternatives?)
4. **Layer 4: Use Cases** (When should I use this?)

**This is the 4th principle** from documentation-exploration: Progressive layers.

---

### Applying Context Chunking

**Scenario**: Exploring Django documentation (100+ pages)

**Layer 1: Overview** (5 minutes)
```
Read: docs/intro/overview.md
Goal: What is Django? What problem does it solve?
Questions: Q1, Q2 from documentation-exploration skill
```

**Layer 2: Core Concepts** (15 minutes)
```
Read: docs/topics/models.md, docs/topics/views.md, docs/topics/templates.md
Goal: What are the core abstractions (Models, Views, Templates)?
Questions: Q3 from documentation-exploration skill
```

**Layer 3: Comparisons** (10 minutes)
```
Read: docs/faq/general.md (Django vs Flask vs FastAPI)
Goal: What design decisions differentiate Django?
Questions: Q2, Q4 from documentation-exploration skill
```

**Layer 4: Use Cases** (10 minutes)
```
Read: docs/intro/tutorial01.md (Quick start)
Goal: What scenarios suit Django? What constraints exist?
Questions: Q4, Q5 from documentation-exploration skill
```

**Total time**: 40 minutes for comprehensive conceptual understanding (not 4 hours reading linearly).

---

## Concept 4: Designing the markdown-generation Skill

### Step 1: Recognize the Recurring Pattern

You've generated markdown documents for:
- README files (project documentation)
- API documentation (endpoint specs)
- Tutorial content (educational material)

**Ask**: What's the SAME across all markdown generation tasks?

**Same patterns**:
- You need to define audience (who reads this?)
- You need to define purpose (what problem does this solve?)
- You need to prioritize minimal essential content (not comprehensive brain-dumps)
- You need to structure for comprehension (headings, bullets, examples)

**Different content**:
- READMEs: Architecture, setup, usage
- API docs: Endpoints, request/response, auth
- Tutorials: Step-by-step learning, exercises

**Insight**: The generation PROCESS is reusable. The content TYPE is not.

---

### Step 2: Extract the Reusable Pattern

**Write the skill**:

```markdown
# markdown-generation

## Persona
"Think like a technical writer creating production documentation that serves
readers efficiently, not comprehensive brain-dumps"

## Questions
1. Who is the reader (audience proficiency)?
2. What problem does this document solve for them?
3. What's the minimum information needed?
4. How can structure guide comprehension?
5. What examples demonstrate concepts effectively?

## Principles
1. Reader-first: Optimize for comprehension
2. Minimal essential: Every section serves learning objective
3. Progressive disclosure: Simple first, complex after foundation
4. Falsifiable quality: Can reader complete task after reading?
5. Structural clarity: Headings, bullets, code blocks
```

---

### Step 3: Test Across Domains

**Domain Test 1: README for REST API Project**

**Apply skill**:
- Q1: Audience = Backend engineers familiar with REST APIs
- Q2: Problem = Need to set up and run API locally for development
- Q3: Minimum info = Installation, configuration, running, testing
- Q4: Structure = Quick Start → Detailed Sections → Advanced
- Q5: Examples = curl commands for each endpoint

**Generated sections**:
- Quick Start (3 commands to run locally)
- Installation (dependencies, setup)
- Configuration (environment variables)
- Running (development vs production)
- API Endpoints (table with examples)
- Testing (how to run tests)

**Result**: ✅ README serves reader's goal (get API running quickly)

---

**Domain Test 2: API Documentation**

**Apply skill**:
- Q1: Audience = Frontend developers integrating with API
- Q2: Problem = Need to understand available endpoints and authentication
- Q3: Minimum info = Endpoints, auth, request/response examples, errors
- Q4: Structure = Authentication → Endpoints Table → Detailed Examples
- Q5: Examples = Request/response JSON for each endpoint

**Generated sections**:
- Authentication (Bearer token format)
- Endpoints (table: Method, Path, Description)
- Request Examples (with auth headers)
- Response Examples (success + error)
- Error Codes (400, 401, 403, 404, 500)

**Result**: ✅ API docs serve reader's goal (integrate successfully)

---

**Domain Test 3: Tutorial for Git Workflow**

**Apply skill**:
- Q1: Audience = Junior developers learning git
- Q2: Problem = Need step-by-step workflow for feature development
- Q3: Minimum info = Branch → Commit → Push → PR → Merge
- Q4: Structure = Concept → Walkthrough → Practice → Validation
- Q5: Examples = Actual commands with expected outputs

**Generated sections**:
- Understanding Branches (concept)
- Step-by-Step Workflow (5 steps)
- Practice Exercise (create feature branch)
- Validation (checklist)
- Troubleshooting (common errors)

**Result**: ✅ Tutorial serves reader's goal (learn workflow through practice)

---

### Reusability Validation

**The markdown-generation skill worked across**:
- README documentation (setup/usage)
- API documentation (technical reference)
- Tutorial content (educational material)

**Same 5 questions, same process, different document types.**

**Conclusion**: This skill is **domain-agnostic** and will transfer to:
- Python project documentation (Part 4)
- Code architecture docs (Part 6)
- Agent system specifications (Part 7+)

---

## Concept 5: Decision Frameworks vs Rigid Rules

### Why Principles Are Frameworks, Not Rules

**Rigid rule** (doesn't account for context):
> "Every README must have exactly these sections: Installation, Usage, Configuration, Testing, Contributing"

**Decision framework** (guides judgment):
> "Reader-first: Include sections that serve reader's primary goal. For developer tools, prioritize getting started quickly. For libraries, prioritize examples and API reference."

**The difference**:
- **Rules** say WHAT to do (prescriptive)
- **Frameworks** say HOW to decide (guidance)

---

### Applying Decision Frameworks

**Scenario**: You're generating a README for a CLI tool.

**Principle 2: Minimal essential content**

**Decision framework thinking**:
- What's the reader's goal? → Run CLI commands successfully
- What's essential for that goal? → Installation, basic commands, examples
- What's NOT essential? → Architecture details, contribution guidelines (most users don't contribute)

**Application**:
- ✅ Include: Quick Start, Installation, Commands, Examples
- ❌ Exclude: Internal architecture, development setup (create separate CONTRIBUTING.md)

---

**Principle 3: Progressive disclosure**

**Decision framework thinking**:
- What does beginner need first? → One command to try the tool
- What does intermediate user need? → All commands with flags
- What does advanced user need? → Configuration, scripting, extending

**Application**:
- Section 1: Quick Start (one command)
- Section 2: Common Commands (5-10 commands)
- Section 3: Advanced Usage (configuration, scripting)
- Appendix: All flags reference

---

## Self-Assessment: Skill Design Practice

### Exercise 1: Recognize Reusable Patterns

**For each scenario below, identify if the pattern is reusable (R) or domain-specific (D)**:

**A**: "Generate Python function docstrings following PEP 257"
- Pattern: ___ (R or D?)
- Reasoning: ___

**B**: "Extract key insights from any technical blog post"
- Pattern: ___ (R or D?)
- Reasoning: ___

**C**: "Debug Django ORM query performance issues"
- Pattern: ___ (R or D?)
- Reasoning: ___

**D**: "Create consistent git commit messages following team conventions"
- Pattern: ___ (R or D?)
- Reasoning: ___

---

### Exercise 2: Design a Skill

**Scenario**: You frequently need to review code for security vulnerabilities. You want to create a reusable skill.

**Design the skill using Persona + Questions + Principles**:

**Persona**: "Think like ___________"

**Questions**:
1. ___________
2. ___________
3. ___________
4. ___________
5. ___________

**Principles**:
1. ___________
2. ___________
3. ___________

---

### Exercise 3: Test Reusability

**Given the skill you designed in Exercise 2, test across these domains**:

**Domain 1**: Review authentication code (Python)
- Apply Q1: ___________
- Apply Q2: ___________

**Domain 2**: Review database queries (SQL)
- Apply Q1: ___________
- Apply Q2: ___________

**Does your skill work for both domains? If not, refine Persona/Questions/Principles.**

---

### Answer Key (Self-Check)

**Exercise 1**:
- A: D (domain-specific — PEP 257 is Python-only)
- B: R (reusable — applies to any technical blog)
- C: D (domain-specific — Django ORM only)
- D: R (reusable if pattern is "consistent messages", D if specific to one team's format)

**Exercise 2 (suggested answer)**:

**Persona**: "Think like a security auditor evaluating code for vulnerabilities through threat modeling, not surface-level checks"

**Questions**:
1. What are the attack surfaces in this code?
2. What threat models apply (injection, XSS, CSRF, auth bypass)?
3. What input validation exists? Where is it missing?
4. What's the likelihood × impact for each vulnerability?
5. How would I prioritize fixes (high risk first)?

**Principles**:
1. Threat model first (understand attack vectors before looking for bugs)
2. Likelihood × impact focus (prioritize high-risk issues)
3. Defense in depth (multiple layers, not single point of failure)
4. Assume breach (how to minimize damage if compromised)

---

## Try With AI

**Setup**: Open Claude Code or Gemini CLI. Choose a framework or library you want to explore (or generate documentation for).

**Exercise**: Apply documentation-exploration and markdown-generation skills.

---

### Practice Task 1: Documentation Exploration

**Goal**: Apply documentation-exploration skill to a new framework (not FastAPI/Django/Flask — choose React, PostgreSQL, or any tool you're curious about).

**Prompts to Try**:

**Prompt 1: Apply the Skill**
```
I want to understand [Framework Name] conceptually.

Apply the documentation-exploration skill:

Persona: "Think like a technical researcher extracting architectural patterns,
not just feature lists"

Questions:
1. What problem does this framework solve?
2. What design decisions differentiate it from alternatives?
3. What are the core abstractions?
4. What constraints or limitations exist?
5. How would I evaluate if this fits my use case?

Start with Question 1.
```

**Expected outcome**: AI should focus on problem/purpose (not features).

---

**Prompt 2: Progressive Layers**
```
Apply context chunking:

Layer 1 (Overview): What is [Framework] and why does it exist?
Layer 2 (Core Concepts): What are the 3-5 key abstractions?
Layer 3 (Comparisons): How does this differ from [Alternative Framework]?
Layer 4 (Use Cases): When should I use this vs alternatives?

Explore Layer 1 first, then ask for next layer.
```

**Expected outcome**: AI should explore in layers, not dump all information at once.

---

### Practice Task 2: Markdown Generation

**Goal**: Apply markdown-generation skill to create documentation.

**Prompts to Try**:

**Prompt 1: Apply the Skill**
```
I need to generate [README/API docs/Tutorial] for [Project].

Apply the markdown-generation skill:

Persona: "Think like a technical writer creating production documentation
that serves readers efficiently"

Questions:
1. Who is the reader (proficiency level)?
2. What problem does this document solve for them?
3. What's the minimum information needed?
4. How can structure guide comprehension?
5. What examples demonstrate concepts effectively?

Answer these questions first, then generate the document.
```

**Expected outcome**: AI should ask clarifying questions before generating (not just produce generic template).

---

**Prompt 2: Iterate with Principles**
```
Review the generated documentation against these principles:

1. Reader-first: Does every section serve the reader's goal?
2. Minimal essential: Is anything included that doesn't map to learning objectives?
3. Progressive disclosure: Does structure go simple → complex?
4. Falsifiable quality: Can reader complete their task after reading?
5. Structural clarity: Are headings, bullets, code blocks used effectively?

Identify gaps and refine.
```

**Expected outcome**: AI should critique its own output and suggest specific improvements.

---

### Challenge: Create Your Own Skill

**Goal**: Design a reusable skill for a pattern you've encountered 2+ times.

**Steps**:

1. **Identify recurring pattern**: What task do you repeat frequently?
2. **Extract decision points**: What questions do you ask each time?
3. **Write skill structure**:

```markdown
# [skill-name]

**Persona**: "Think like [role/approach]"

**Questions**:
1. [Analytical question 1]
2. [Analytical question 2]
3. [Analytical question 3]
4. [Analytical question 4]
5. [Analytical question 5]

**Principles**:
1. [Decision framework 1]
2. [Decision framework 2]
3. [Decision framework 3]
```

4. **Test across 3 domains**: Does this work beyond original context?
5. **Refine**: Adjust Persona/Questions/Principles based on testing

---

### Safety Note

**Validate skill outputs**: Always verify that:
- **Skills are domain-agnostic** (not overfitted to one framework/tool)
- **Questions guide reasoning** (not prescriptive checklists)
- **Principles are frameworks** (not rigid rules)
- **Skills transfer across contexts** (test in 3+ domains)

Reusable skills are powerful, but you remain responsible for validating that they produce accurate, context-appropriate outputs.

---

**Lesson Metadata**:
- **Stage**: 3 (Intelligence Design)
- **Modality**: Specification-first + skill creation
- **Concepts**: 5 (Persona+Questions+Principles pattern, context chunking, decision frameworks, reusability testing, skill validation)
- **Cognitive Load**: B1 tier (5 ≤ 10)
- **AI Tools**: Claude Code OR Gemini CLI (platform-agnostic)
- **Duration**: 90 minutes
- **Skills Created**: documentation-exploration, markdown-generation (Persona + Questions + Principles)
- **Version**: 1.0.0
- **Constitution**: v6.0.0 Compliance
- **Generated**: 2025-01-18
- **Source Spec**: `specs/025-chapter-10-redesign/spec.md`
