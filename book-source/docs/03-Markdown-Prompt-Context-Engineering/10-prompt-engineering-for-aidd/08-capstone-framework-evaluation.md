# Lesson 8: Capstone — Systematic Framework Evaluation (Spec-Driven Integration)

---

## What Should This Lesson Enable You to Do?

By the end of this lesson, you should be able to:

- **Write specifications FIRST** before executing any analysis (intent, success criteria, constraints, non-goals)
- **Compose accumulated intelligence** from Lessons 1-7 (debugging-protocol, documentation-exploration, markdown-generation)
- **Orchestrate systematic exploration** using AI tools strategically
- **Produce professional deliverables** (2-page framework comparison report)
- **Validate outputs against specifications** (ensure deliverable meets stated criteria)

**Success criteria**: You can evaluate three Python web frameworks (FastAPI, Django, Flask) by writing a specification first, composing skills from previous lessons, and producing a 2-page comparison report that meets professional standards.

---

## The Capstone Challenge

### The Scenario

You are a **VP of Engineering** making a critical architectural decision: Your company is rebuilding its customer-facing API from scratch, and you need to choose the right Python web framework. The decision affects:
- **15 engineers** (team productivity for 2+ years)
- **$500K investment** (development + migration + training)
- **6-month timeline** (market window closing)

Your CEO wants evidence-based recommendation in **2 hours**.

**You must evaluate**:
1. **FastAPI** (modern, async, type-safe)
2. **Django** (batteries-included, ORM, admin)
3. **Flask** (minimal, flexible, extensions)

**Deliverable**: 2-page technical assessment covering:
- **Page 1**: Architecture comparison (design decisions, trade-offs)
- **Page 2**: Strategic recommendation (which framework for which scenarios)

---

### Why This Is Spec-Driven Integration (Stage 4)

**You won't learn new content in this lesson.** Instead, you'll **compose everything from Lessons 1-7**:

**From Lesson 1-2**: Specification-first thinking (define WHAT before HOW)
**From Lesson 3**: 4-layer context model (Project → Code → Constraints → Analyst)
**From Lesson 4**: Claude Code tools (Read, WebFetch, Grep for documentation)
**From Lesson 5**: Gemini CLI workflows (file references, custom commands)
**From Lesson 6**: debugging-protocol skill (systematic troubleshooting if issues arise)
**From Lesson 7**: documentation-exploration + markdown-generation skills (explore frameworks, generate report)

**This is orchestration, not addition.** You're proving you can integrate accumulated intelligence systematically.

---

## Concept 1: Specification-First Workflow (WHAT Before HOW)

### Why Write Spec Before Analysis?

**Without spec** (reactive approach):
1. Start exploring FastAPI docs randomly
2. Get distracted by interesting features
3. Realize 90 minutes later you haven't compared to Django yet
4. Panic, write rushed comparison
5. Deliver incomplete report

**With spec** (proactive approach):
1. Write specification defining success criteria (10 minutes)
2. Execute analysis systematically against spec (80 minutes)
3. Validate deliverable meets all criteria (10 minutes)
4. Deliver complete, evidence-based report

**The spec is your contract with yourself.** It prevents scope creep and ensures completeness.

---

### Capstone Specification Template

**Write this BEFORE starting any exploration:**

```markdown
# Framework Evaluation Specification

## Intent (The "Why")

**Decision needed**: Choose Python web framework for new customer-facing API

**Stakeholders**: CEO (business impact), CTO (technical strategy), engineering team (implementation)

**Timeline**: 2 hours to recommendation

**Impact**: $500K investment, 15 engineers, 2+ year commitment

## Success Criteria (Measurable Outcomes)

**Page 1: Architecture Comparison**
- ✅ Compare core design decisions for all 3 frameworks
- ✅ Identify key differentiators (what makes each unique)
- ✅ Document trade-offs (what you optimize for, what you sacrifice)
- ✅ Include architecture diagram or table for visual comparison

**Page 2: Strategic Recommendation**
- ✅ Recommend framework for specified use case (customer-facing API)
- ✅ Provide decision framework (when to choose X vs Y vs Z)
- ✅ Identify risks and mitigation strategies
- ✅ Estimate adoption effort (timeline, training, migration)

**Format Requirements**:
- 2 pages maximum (concise, not comprehensive)
- Markdown format (for version control, collaboration)
- Evidence-based claims (cite documentation, not opinions)
- Actionable recommendations (CEO can make decision from this)

## Constraints (The "How Not")

**Technical constraints**:
- Must evaluate async capabilities (API is I/O-bound)
- Must consider type safety (team values static typing)
- Must assess ecosystem maturity (production-grade libraries)

**Time constraints**:
- 2 hours total (including spec writing, exploration, report generation)
- No time for proof-of-concept implementations (documentation only)

**Team constraints**:
- Team: 15 engineers, 8 years Python experience, 0 years FastAPI/Django/Flask production experience
- Learning curve matters (onboarding time affects timeline)

**Non-negotiable requirements**:
- PostgreSQL database (existing infrastructure)
- Kubernetes deployment (existing platform)
- OAuth2 authentication (security standard)

## Non-Goals (Explicit Scope Boundaries)

**Out of scope for this evaluation**:
- ❌ Proof-of-concept implementation (docs only)
- ❌ Performance benchmarking (trust published benchmarks)
- ❌ Full migration plan (separate follow-up)
- ❌ Evaluating non-Python frameworks (Ruby, Go, Node.js)

**Why these are non-goals**:
- 2-hour timeline insufficient for implementation
- Focus on architectural fit, not micro-optimizations
- Migration details need chosen framework first

## Success Validation

**How to know deliverable meets spec**:

**Completeness check**:
- [ ] All 3 frameworks compared (FastAPI, Django, Flask)
- [ ] All success criteria addressed (Page 1 + Page 2 sections)
- [ ] Format requirements met (2 pages, markdown, evidence-based)

**Quality check**:
- [ ] CEO can make decision from this report alone
- [ ] Recommendations are specific (not generic "depends on use case")
- [ ] Evidence cited (documentation links for claims)
- [ ] Trade-offs explicitly stated (not just benefits)

**If any checkbox unchecked → Deliverable incomplete.**
```

---

## Concept 2: Composing Accumulated Intelligence

### Mapping Skills to Capstone Tasks

**You now have 3 reusable skills** (from Lessons 6-7). The capstone orchestrates them:

**Task 1: Explore Framework Documentation**
→ **Use**: documentation-exploration skill (Lesson 7)
→ **Apply to**: FastAPI, Django, Flask docs
→ **Output**: Conceptual understanding (design decisions, trade-offs, constraints)

**Task 2: Generate Comparison Report**
→ **Use**: markdown-generation skill (Lesson 7)
→ **Apply to**: 2-page framework comparison
→ **Output**: Structured markdown document meeting spec requirements

**Task 3: Debug Issues (If Needed)**
→ **Use**: debugging-protocol skill (Lesson 6)
→ **Apply to**: Any errors during exploration (broken links, rendering issues, tool failures)
→ **Output**: Systematic troubleshooting, root cause identification

---

### Orchestration Workflow

**Step 1: Write Specification** (10 minutes)
- Use capstone spec template above
- Define success criteria BEFORE starting
- Identify constraints and non-goals

**Step 2: Explore Framework 1 (FastAPI)** (20 minutes)
- Apply documentation-exploration skill:
  - Q1: What problem does FastAPI solve?
  - Q2: What design decisions differentiate it?
  - Q3: What are core abstractions?
  - Q4: What constraints exist?
  - Q5: How to evaluate fit?
- Use Claude Code tools (Read, WebFetch, Grep) OR Gemini CLI (@filename, !command)
- Document findings in notes

**Step 3: Explore Framework 2 (Django)** (20 minutes)
- Apply documentation-exploration skill (same 5 questions)
- Use same tools as Step 2
- Document findings

**Step 4: Explore Framework 3 (Flask)** (20 minutes)
- Apply documentation-exploration skill (same 5 questions)
- Use same tools
- Document findings

**Step 5: Generate Comparison Report** (20 minutes)
- Apply markdown-generation skill:
  - Q1: Audience = CEO + CTO + engineering team
  - Q2: Problem = Need framework decision
  - Q3: Minimum info = Architecture comparison + recommendation
  - Q4: Structure = Page 1 (comparison), Page 2 (recommendation)
  - Q5: Examples = Decision framework table, trade-offs

**Step 6: Validate Against Spec** (10 minutes)
- Check all success criteria met
- Verify format requirements (2 pages, markdown, evidence-based)
- Confirm CEO can make decision from report

**Total**: 100 minutes (20 minutes buffer for unexpected issues)

---

## Concept 3: The 2-Page Report Format

### Page 1: Architecture Comparison

**Structure**:

```markdown
# Framework Evaluation: FastAPI vs Django vs Flask

## Executive Summary (2 sentences)
[State decision and rationale in 2 sentences]

## Architecture Comparison

| Framework | Core Design Decision | Key Strength | Key Limitation |
|-----------|---------------------|--------------|----------------|
| FastAPI   | Async + Type Hints  | Performance, Type Safety | Learning Curve (Async) |
| Django    | Batteries-Included ORM | Rapid Development, Admin | Monolithic, Opinionated |
| Flask     | Minimal Core + Extensions | Flexibility, Simplicity | Manual Feature Addition |

## Design Philosophy

### FastAPI
**Problem solved**: High-performance async APIs with type safety
**Differentiator**: Pydantic models for automatic validation
**Abstraction**: Path operations, dependency injection, async/await
**Constraint**: Python 3.7+, async learning curve

### Django
**Problem solved**: Full-stack web development with admin interface
**Differentiator**: ORM-first, batteries-included (auth, admin, migrations)
**Abstraction**: Models, Views, Templates (MTV), Admin
**Constraint**: Opinionated structure, harder to customize

### Flask
**Problem solved**: Lightweight web development with control
**Differentiator**: Minimal core, extension ecosystem
**Abstraction**: Routes, Views, Blueprints, Context
**Constraint**: Manual feature addition (auth, ORM, migrations)
```

---

### Page 2: Strategic Recommendation

**Structure**:

```markdown
## Recommendation: FastAPI for Customer-Facing API

### Rationale

**Our use case**:
- I/O-bound API (async benefits significant)
- Team values type safety (mypy, IDE autocomplete)
- PostgreSQL database (all frameworks support)
- Kubernetes deployment (all frameworks compatible)

**Why FastAPI fits best**:
1. **Performance**: Async handles 10K concurrent requests (our requirement)
2. **Type Safety**: Pydantic models catch errors pre-runtime (team preference)
3. **Developer Experience**: Auto-generated docs (OpenAPI/Swagger)
4. **Modern Python**: Leverages 3.7+ features (type hints, async)

**Trade-offs accepted**:
- Learning curve: 2-week training vs 1-week for Flask
- Smaller ecosystem: Fewer extensions than Django/Flask
- Async complexity: Requires understanding event loops

### Decision Framework (When to Choose Alternatives)

**Choose FastAPI if**:
- ✅ Need high-performance async APIs
- ✅ Team values type safety
- ✅ Modern Python acceptable (3.7+)

**Choose Django if**:
- ✅ Need full-stack web app with admin interface
- ✅ Rapid prototyping priority
- ✅ ORM-first development preferred

**Choose Flask if**:
- ✅ Want maximum architecture control
- ✅ Building lightweight microservices
- ✅ Minimal framework preferred

### Adoption Plan

**Timeline**: 6 weeks
- Week 1-2: Team training (FastAPI + async patterns)
- Week 3-4: Proof-of-concept (1 endpoint, PostgreSQL, OAuth2)
- Week 5-6: Pilot service (customer preferences API)

**Risks & Mitigation**:
- **Risk**: Async learning curve → **Mitigation**: 2-week training, pair programming
- **Risk**: Smaller ecosystem → **Mitigation**: Vet extensions before adoption
- **Risk**: Debugging async issues → **Mitigation**: Apply debugging-protocol skill

### Validation

**Success criteria met**:
- ✅ Architecture comparison (3 frameworks, design decisions, trade-offs)
- ✅ Strategic recommendation (FastAPI for customer-facing API)
- ✅ Decision framework (when to choose alternatives)
- ✅ Adoption plan (timeline, risks, mitigation)
- ✅ Format (2 pages, markdown, evidence-based)

**CEO can make decision**: Yes (recommendation is specific, rationale clear, risks identified)
```

---

## Concept 4: Validation Against Specification

### The Final Quality Gate

After generating your report, **validate against the spec**:

**Completeness checklist**:
- [ ] All 3 frameworks compared (FastAPI ✅, Django ✅, Flask ✅)
- [ ] Page 1: Architecture comparison with table/diagram
- [ ] Page 1: Design philosophy for each framework
- [ ] Page 2: Specific recommendation (not vague "it depends")
- [ ] Page 2: Decision framework (when to choose alternatives)
- [ ] Page 2: Adoption plan (timeline, risks)
- [ ] Format: 2 pages, markdown, evidence-based

**Quality checklist**:
- [ ] Can CEO make decision from this report alone? (Actionable)
- [ ] Are recommendations specific? (Not "FastAPI is good if...")
- [ ] Are claims cited? (Links to documentation)
- [ ] Are trade-offs explicit? (Not just benefits)

**If any checkbox unchecked → Iterate**:
1. Identify gap (what's missing?)
2. Apply relevant skill (documentation-exploration or markdown-generation)
3. Fill gap
4. Re-validate

---

## Self-Assessment: Capstone Planning Exercise

### Exercise 1: Write Your Own Specification

**Scenario**: You need to evaluate three Python testing frameworks (pytest, unittest, nose2) for your team.

**Write a capstone specification**:

**Intent**:
- Decision needed: ___________
- Stakeholders: ___________
- Timeline: ___________

**Success Criteria**:
- Page 1: ___________
- Page 2: ___________
- Format: ___________

**Constraints**:
- ___________
- ___________

**Non-Goals**:
- ___________
- ___________

---

### Exercise 2: Map Skills to Tasks

**For the testing framework evaluation above, map which skills you'd use**:

**Task**: Explore pytest documentation
- **Skill**: ___________
- **Output**: ___________

**Task**: Generate comparison report
- **Skill**: ___________
- **Output**: ___________

**Task**: Debug broken documentation link
- **Skill**: ___________
- **Output**: ___________

---

### Exercise 3: Validate Report Against Spec

**Given this incomplete report**:
```markdown
# Testing Framework Comparison

pytest is the best testing framework because it's popular.
unittest is built-in but harder to use.
nose2 is outdated.

Recommendation: Use pytest.
```

**Identify what's missing**:
- [ ] Completeness gaps: ___________
- [ ] Quality issues: ___________
- [ ] Spec violations: ___________

---

### Answer Key (Self-Check)

**Exercise 1** (suggested answer):
- Intent: Choose testing framework for Python project (10 engineers, 2-year commitment)
- Stakeholders: Engineering team, Tech Lead
- Success Criteria: Compare 3 frameworks (features, learning curve, ecosystem), recommend best fit
- Constraints: Must support fixtures, parallelization, coverage reporting
- Non-Goals: Performance benchmarking, full test suite migration

**Exercise 2**:
- Explore pytest: documentation-exploration skill → Design decisions, abstractions, constraints
- Generate report: markdown-generation skill → 2-page comparison
- Debug link: debugging-protocol skill → Isolate issue, test hypothesis

**Exercise 3** (missing):
- Completeness: No architecture comparison table, no design philosophy details, no adoption plan
- Quality: Claims not cited ("popular" not evidence), no trade-offs, generic recommendation
- Spec violations: Not 2 pages, no decision framework, not actionable

---

## Try With AI

**Setup**: Open Claude Code or Gemini CLI. Choose three frameworks/libraries you want to evaluate (or use FastAPI/Django/Flask from this lesson).

**Exercise**: Execute the full capstone workflow from specification to validated deliverable.

---

### Capstone Execution: Full Workflow

**Time allocation**: 100 minutes (2 hours with buffer)

---

### Step 1: Write Specification (10 minutes)

**Prompt**:
```
I need to evaluate [Framework A], [Framework B], [Framework C] for [use case].

Help me write a complete specification using this template:

## Intent
- Decision needed: [What decision?]
- Stakeholders: [Who cares?]
- Timeline: [How long?]

## Success Criteria
- Page 1: [What on page 1?]
- Page 2: [What on page 2?]

## Constraints
- [Technical constraint 1]
- [Time/team constraint 2]

## Non-Goals
- [Out of scope 1]
- [Out of scope 2]

Fill in details based on [describe your use case].
```

**Expected outcome**: Complete specification with measurable success criteria.

---

### Step 2-4: Explore Frameworks (60 minutes total, 20 each)

**Prompt** (repeat for each framework):
```
Apply the documentation-exploration skill to [Framework Name]:

**Persona**: "Think like technical researcher extracting architectural patterns,
not just features"

**Questions**:
1. What problem does [Framework] solve?
2. What design decisions differentiate it from [Alternatives]?
3. What are the core abstractions?
4. What constraints or limitations exist?
5. How would I evaluate if this fits [my use case]?

Use [Read/WebFetch/Grep OR @filename/!command] to explore documentation systematically.

Provide answers to all 5 questions with evidence (cite documentation).
```

**Expected outcome**: For each framework, you have answers to 5 questions with documentation citations.

---

### Step 5: Generate Comparison Report (20 minutes)

**Prompt**:
```
Apply the markdown-generation skill to create a 2-page framework comparison:

**Persona**: "Think like technical writer creating production documentation
that serves readers efficiently"

**Questions**:
1. Audience: [CEO, CTO, engineering team]
2. Problem: [Need framework decision]
3. Minimum info: [Architecture comparison + recommendation]
4. Structure: [Page 1: comparison table + design philosophy,
              Page 2: recommendation + decision framework + adoption plan]
5. Examples: [Decision framework table showing when to choose X vs Y vs Z]

Generate the report in markdown following this structure:

# Page 1: Architecture Comparison
[Comparison table]
[Design philosophy for each framework]

# Page 2: Strategic Recommendation
[Specific recommendation with rationale]
[Decision framework (when to choose alternatives)]
[Adoption plan (timeline, risks, mitigation)]
```

**Expected outcome**: 2-page markdown report meeting specification.

---

### Step 6: Validate Against Spec (10 minutes)

**Prompt**:
```
Validate this report against the specification:

**Specification success criteria**:
[Paste your success criteria from Step 1]

**Report**:
[Paste your generated report from Step 5]

Check:
- [ ] All success criteria met?
- [ ] Format requirements met (2 pages, markdown, evidence-based)?
- [ ] Can decision-maker act on this report alone?
- [ ] Are recommendations specific (not vague)?
- [ ] Are claims cited (documentation links)?
- [ ] Are trade-offs explicit?

Identify gaps and suggest specific improvements.
```

**Expected outcome**: Validation checklist with gap identification and refinement suggestions.

---

### Challenge: Iteration Loop

**If validation reveals gaps**:

1. **Identify gap**: What's missing? (e.g., "No adoption plan")
2. **Apply skill**: Which skill fixes this? (markdown-generation for missing section)
3. **Refine**: Generate missing content
4. **Re-validate**: Check if gap filled

**Iterate until all checkboxes checked.**

---

### Safety Note

**Validate capstone outputs**: Always verify that:
- **Specifications are complete** (intent, success criteria, constraints, non-goals)
- **Skills are composed correctly** (right skill for right task)
- **Reports are evidence-based** (claims cite documentation)
- **Deliverables meet specifications** (all success criteria addressed)

The capstone tests your ability to orchestrate intelligence systematically. Validation ensures quality.

---

## Reflection: What You've Accomplished (Lessons 1-8)

### Stage 1 (Lessons 1-2): Manual Foundation
- ✅ Learned how AI agents reason (context windows, token generation, mental models)
- ✅ Learned specification-first thinking (WHAT before HOW)
- ✅ Built foundation WITHOUT AI tools (mental models first)

### Stage 2 (Lessons 3-5): AI Collaboration
- ✅ Learned 4-layer context model (Project → Code → Constraints → Analyst)
- ✅ Demonstrated Three Roles (AI as Teacher, Student, Co-Worker)
- ✅ Mastered Claude Code tools (Read, WebFetch, Grep)
- ✅ Mastered Gemini CLI workflows (@filename, !command, custom TOML)

### Stage 3 (Lessons 6-7): Intelligence Design
- ✅ Created debugging-protocol skill (systematic troubleshooting)
- ✅ Created documentation-exploration skill (framework evaluation)
- ✅ Created markdown-generation skill (production documentation)
- ✅ Tested skills across domains (markdown, bash, git, frameworks)

### Stage 4 (Lesson 8): Spec-Driven Integration
- ✅ Orchestrated accumulated intelligence (composed 3 skills)
- ✅ Produced professional deliverable (2-page framework comparison)
- ✅ Validated against specification (all success criteria met)

---

## What Transfers to Part 4 (Python Coding)

**These patterns apply DIRECTLY to coding**:

**Specification-first** (Lessons 1-2):
- Write spec.md BEFORE writing code
- Define intent, success criteria, constraints
- Validate code against spec

**4-layer context** (Lesson 3):
- Provide context to AI when generating code
- Project context (what are we building?)
- Code context (what exists already?)
- Constraints context (what are limits?)
- Analyst context (who am I, what do I need?)

**Debugging protocol** (Lesson 6):
- Debug Python errors systematically
- Isolate symptom → hypothesize → test → validate
- SAME protocol, different domain (Python vs markdown)

**Documentation exploration** (Lesson 7):
- Evaluate Python libraries (Pandas, NumPy, SQLAlchemy)
- Same 5 questions, different domain

**Markdown generation** (Lesson 7):
- Generate Python project READMEs, module docs
- Same 5 questions, different document type

**Spec-driven development** (Lesson 8):
- Spec → Compose skills → Generate code → Validate
- SAME workflow, different output (code vs report)

---

**Lesson Metadata**:
- **Stage**: 4 (Spec-Driven Integration)
- **Modality**: Specification-first + composition
- **Concepts**: 0 new (orchestration only)
- **Cognitive Load**: B1 tier (0 new concepts, integration of L1-7)
- **AI Tools**: Claude Code OR Gemini CLI (platform-agnostic) + All 3 skills
- **Duration**: 120 minutes (includes full capstone execution)
- **Skills Composed**: debugging-protocol, documentation-exploration, markdown-generation
- **Version**: 1.0.0
- **Constitution**: v6.0.0 Compliance
- **Generated**: 2025-01-18
- **Source Spec**: `specs/025-chapter-10-redesign/spec.md`
