---
title: "Mini-Project 2: Write a Production Grading System Spec"
chapter: 31
lesson: 9
duration: "3-4 hours"
skills:
  - name: "Production Specification"
    proficiency: "B1"
    category: "Technical"
  - name: "Enterprise System Design"
    proficiency: "B1"
    category: "Technical"
  - name: "Non-Functional Requirements"
    proficiency: "B1"
    category: "Technical"
learning_objectives:
  - "Write production-grade specifications with security, performance, and reliability requirements (B1)"
  - "Define non-functional constraints and acceptance criteria (B1)"
  - "Structure a specification suitable for team implementation (B1)"
---

# Write a Production Grading System Spec

## What You're About to Do

Mini-Project 1 was about building code from a spec. This project is about **writing a production-grade specification for a real-world problem**—without implementation.

This is a critical skill: **specification writing is separate from coding.** Professional architects write specs without knowing who will code them or what language they'll use. The spec is a contract: "This is what must be built."

You're going to write a specification for a **production grading system inspired by real companies** (Gradewiz, Edexia, Frizzle, Mimir). You'll apply all the skills from Chapter 26:

- Coaching clarity
- Writing SMART criteria
- Addressing error handling and edge cases
- Thinking like an architect, not a coder

By the end, you'll have a **specification that a team of developers could hand off to build**, OR that you could hand to an AI to implement with high confidence.

---

## Part 1: Research the Real Problem

Before you write specs, understand the real problem. **Ask your companion:**

```
I want to understand the real grading problem that tools like
Gradewiz and Edexia solve. Can you tell me:

1. What problem do teachers actually have with grading?
2. What tools do they currently use (spreadsheets? LMS?)
3. What takes the most time?
4. What causes inconsistency?
5. What features would help them most?

Don't tell me what software exists. Tell me what the problem is.
```

Your companion will explain something like:

> "Real grading challenges:
>
> 1. **Time**: Teachers spend 2-4 hours grading 30 essays. That's 4-8 minutes per student.
> 2. **Inconsistency**: How do you consistently apply 'organization is good' across 30 essays?
> 3. **Feedback**: Generic feedback ('Good work!') doesn't help students improve.
> 4. **Tracking**: Spreadsheets are bad at tracking rubric-based grades across multiple assignments.
> 5. **Adjustment**: Changing one grade or rubric requires manual updates everywhere.
>
> Existing tools:
>
> - Spreadsheets: Time-consuming, error-prone
> - LMS built-in tools: Too simple, limited rubric support
> - Manual paper: Slow, no digital record
>
> What would help:
>
> - Pre-built rubrics they can customize
> - Automatic essay analysis (not AI, but text metrics)
> - Bulk grading workflows
> - Grade analytics (who's struggling?)
> - Integration with existing LMS"

---

## Part 2: Define Your Scope (The Real Grading System)

You're now going to write a **production-grade specification** that addresses these real problems.

**Start a new SpecifyPlus project:**

```bash
uvx specifyplus init grading-system-production
cd grading-system-production
```

But first, **coach clarity with your companion** on your own vision. You're going to write something more sophisticated than the Lesson 7 example:

```
I want to build a production grading system for teachers.
It should address real problems: saving time, maintaining consistency,
providing meaningful feedback.

But I need to be realistic about scope. I can't build everything.

Help me think through:

1. Who exactly is my first user? (K-12? College? What subject?)
2. What's my absolute must-have feature set?
3. What can I defer to later phases?
4. What's the realistic deadline/effort?
5. How is this different from existing tools?
```

Your companion will help you narrow scope. For this exercise, let's assume:

- **User**: College professors grading essays and programming assignments
- **Must-have**: Rubric-based grading with built-in rubric templates
- **Phase 2**: AI-powered feedback generation
- **Competitive advantage**: Fastest grading workflow + rubric recommendations based on course data

---

## Part 3: Write Your Production Specification

Now open `specs/grading-system-production/spec.md` and write a comprehensive specification that:

1. Addresses the real problem (not just "I want a calculator")
2. Has realistic scope (must-have vs. future phases)
3. Is production-ready (professional team could bid on building this)

**Here's a production-grade example you can adapt:**

```markdown
---
title: "RapidGrade: Production Grading System for Educators"
feature: "rapidgrade-production"
version: "1.0"
status: "Ready for Implementation Planning"
---

# Specification: RapidGrade – Production Grading System

## Executive Summary

RapidGrade is a rubric-based grading platform designed to help
educators grade assignments 3-5x faster while maintaining consistency
and providing meaningful feedback to students.

**Target User**: College and high school instructors
**MVP Deliverable**: Grading workflow + 50 pre-built rubric templates
**Timeline**: 16-20 weeks for MVP
**Market**: Education technology, instructor productivity

---

## 1. Problem Statement

### Current State

Educators spend 2-4 hours per assignment batch:

- Manual rubric application (inconsistent)
- Writing similar feedback repeatedly (40% of time)
- Grade tracking across assignments and semesters (spreadsheets, error-prone)

### Desired State

Educators can grade the same batch in 30-45 minutes:

- Structured rubric application (consistent)
- Template-based feedback (customizable)
- Integrated grade tracking (analytics ready)

### Market Validation

- 3.7 million K-12 teachers in US
- 1.2 million college instructors
- Current solutions (Canvas, Blackboard, Google Classroom) lack dedicated grading
- Competitor products (Gradewiz, Edexia, Mimir) prove market demand

---

## 2. Product Overview

RapidGrade is a web-based grading platform offering:

- **Rubric Library**: 50+ pre-built rubrics (customizable)
- **Fast Grading Workflow**: Click-to-apply rubric criteria
- **Template Feedback**: AI-free feedback templates (reduces writing by 60%)
- **Grade Analytics**: Student progress tracking, trend analysis
- **Export & Integration**: CSV export, LMS integration (Phase 2)

**Key Differentiator**: Fastest grading workflow + recommended rubrics (from classroom data)

---

## 3. Users & Use Cases

### Primary User: Professor Sarah (College)

- **Role**: Literature professor, 150 students across 3 sections
- **Pain**: Grades 50 essays/week (3 hours); wants to grade faster while being fair
- **Success**: Grades 50 essays in 1 hour; feedback is specific and helps students

### Secondary User: Mr. Chen (High School)

- **Role**: Math teacher, 120 students across 4 sections
- **Pain**: Grades code assignments (100+ lines each); wants consistency
- **Success**: Can grade 30 programs in 45 minutes with consistent rubric application

### Tertiary User: Dept Chair (Admin)

- **Role**: Department administrator
- **Pain**: Wants visibility into grading consistency across instructors
- **Success**: Can generate reports showing how different instructors apply rubrics

---

## 4. Core Features (MVP)

### 4.1 Rubric Management

**Capability**: Create, customize, and apply rubric templates

**Features**:

- Browse 50+ pre-built rubrics (essays, code, presentations, math)
- Create new rubric with 3-5 criteria
- Define score levels per criterion (1-4 stars or 0-100 points)
- Add descriptors ("Excellent", "Good", "Needs Work") per level
- Save rubrics to personal library
- Duplicate and modify existing rubrics
- Share rubrics with co-instructors

**Technical Requirements**:

- Rubric data model with criteria, levels, descriptors
- Rubric versioning (changes don't affect past grades)
- Search/filter across rubric library
- Rubric templates loadable from database

**Acceptance Criteria**:

- [ ] Can load a pre-built rubric in < 5 seconds
- [ ] Can customize a rubric in < 2 minutes
- [ ] Can save custom rubric and reuse in future assignments
- [ ] Rubric changes apply only to new grades (not retroactive)
- [ ] Search finds rubrics by keyword (e.g., "essay analysis")

### 4.2 Assignment Grading Workflow

**Capability**: Grade submissions quickly using rubric

**Features**:

- Upload assignment submissions (batch upload 30+ files)
- Display student submission + rubric side-by-side
- Click rubric criteria to apply
- Auto-calculate total score
- Add notes or custom feedback
- Navigation: Previous/Next student
- Save grade and move to next submission

**Technical Requirements**:

- File handling (essays: .pdf, .docx, .txt; code: .py, .js, .java, etc.)
- Side-by-side UI (submission left, rubric right)
- Real-time score calculation
- Draft/saved state tracking

**Acceptance Criteria**:

- [ ] Can grade a submission in < 60 seconds (including rubric review)
- [ ] Grading 30 submissions takes < 30 minutes
- [ ] Score calculates automatically from criteria
- [ ] Can navigate between submissions without losing data
- [ ] Grades persist (recovered if page refreshes)

### 4.3 Grade & Feedback Management

**Capability**: Store grades, generate feedback, track history

**Features**:

- Automatic feedback generation from selected criteria
- Template-based feedback (not AI)
- Manual feedback editing and customization
- Grade history (original, edits, timestamps)
- Bulk feedback application (e.g., "All who scored < 70 get template X")
- Grade export (CSV, PDF)

**Technical Requirements**:

- Feedback template library (associated with rubrics)
- Grade versioning (audit trail)
- Bulk operations support
- Export formatting

**Acceptance Criteria**:

- [ ] Feedback auto-generated from rubric selection (no manual typing for standard cases)
- [ ] Teacher can customize feedback in < 30 seconds
- [ ] Grade changes tracked with before/after values and timestamp
- [ ] Can apply same feedback to multiple students
- [ ] Grades exportable to CSV

### 4.4 Analytics Dashboard

**Capability**: Visualize grading patterns and student progress

**Features**:

- Grade distribution chart (histogram)
- Student progress over time
- Criteria-level analytics (which criteria are struggling points?)
- Instructor comparison (how do my rubric applications compare to department average?)
- Custom report builder

**Technical Requirements**:

- Aggregation queries on grade data
- Visualization library (Chart.js or similar)
- Report export (PDF)

**Acceptance Criteria**:

- [ ] Dashboard loads in < 2 seconds
- [ ] Can view grade distribution for any assignment
- [ ] Can see which criteria most students struggle with
- [ ] Can compare this assignment's grades to previous semesters

---

## 5. Technical Architecture

### Tech Stack

- **Frontend**: React (or Vue.js), TypeScript
- **Backend**: Python (FastAPI) or Node.js (Express)
- **Database**: PostgreSQL (relational for rubrics, grades)
- **Storage**: S3 or similar for file uploads
- **Deployment**: Docker + Kubernetes (or simpler: AWS ECS)

### Key Data Models
```

User: id, email, institution, role (teacher/admin)
Rubric: id, name, owner_id, criteria[], version, created_date
Criterion: id, name, levels[score, descriptor]
Assignment: id, course_id, name, rubric_id, due_date
Submission: id, assignment_id, student_id, file_url, uploaded_date
Grade: id, submission_id, rubric_id, score, feedback, created_date, updated_date

```

### Performance Budgets
- Page load: < 2 seconds
- Grading UI interaction: < 500ms
- Grade save: < 1 second
- Dashboard load: < 3 seconds
- Rubric search: < 1 second

### Scalability
- MVP: 1,000 instructors, 100,000 students
- Phase 2 (12 months): 10,000 instructors, 1M students
- No artificial limits on submissions per assignment

---

## 6. Out-of-Scope (Phase 2+)

**Explicitly deferred:**
- AI-powered feedback generation
- LMS integration (Canvas, Blackboard, Google Classroom)
- Student-facing feedback portal
- Mobile app
- Plagiarism detection
- Attendance tracking
- Gradebook sync with institutional systems

---

## 7. Success Metrics

The product is successful when:

1. **Time Savings**: Teachers grade 3-5x faster (validated by user testing)
2. **Consistency**: Same submission graded by same rubric produces consistent feedback
3. **Engagement**: Teachers use system for ≥80% of assignments after onboarding
4. **Satisfaction**: Net Promoter Score ≥ 50 (survey)
5. **Reliability**: 99.5% uptime (no unplanned outages)
6. **Retention**: 60% of sign-ups maintain active use after 3 months

---

## 8. Constraints & Assumptions

### Technical Constraints
- Browser-based (no desktop app initially)
- Supports files up to 50MB
- Concurrent users: < 100 per institution (not massive collaborative use)
- US deployment only (Phase 1)

### Assumptions
- Teachers know what good rubric criteria look like (we provide templates)
- File submissions are single files per student (not zip archives)
- Grades are numeric (not letter grades initially)
- Teachers manage their own institution accounts (no SSO Phase 1)

---

## 9. Risks & Mitigation

| Risk | Impact | Mitigation |
|------|--------|-----------|
| Rubric templates don't fit teachers' needs | Low adoption | User testing with 20+ teachers; customization features |
| Performance degrades with 100K+ submissions | User churn | Database indexing; async processing; caching |
| Teachers don't like UI (takes longer than current) | Product failure | Iterative UI testing; compare to spreadsheet baseline |
| File handling bugs cause data loss | Legal/trust | Comprehensive file handling tests; automated backups |

---

## 10. Implementation Phases

### Phase 1 (16 weeks): MVP
- Rubric management (50 templates)
- Grading workflow
- Grade storage
- Basic analytics
- Testing and documentation

### Phase 2 (12 weeks): AI & Integration
- AI-powered feedback suggestions
- LMS integration
- Student feedback portal
- Plagiarism detection

### Phase 3 (Ongoing): Scale & Insights
- Mobile app
- Advanced analytics
- Multi-institutional deployment

---

## 11. Acceptance Criteria (QA Checklist)

**Product Acceptance**:
- [ ] 50 pre-built rubrics loaded and tested
- [ ] Can upload 30+ files in one batch
- [ ] Grading interface responsive (< 500ms interactions)
- [ ] Grades persist across sessions
- [ ] Feedback auto-generates from criteria
- [ ] Dashboard shows grade distribution and analytics
- [ ] All error cases handled with clear messages
- [ ] > 80% test coverage
- [ ] Documentation complete (setup, usage, admin)
- [ ] User testing with 5+ real teachers shows 3-5x speed improvement

**Infrastructure Acceptance**:
- [ ] Deployed to production (AWS or similar)
- [ ] SSL/TLS enabled
- [ ] Database backups automated daily
- [ ] Monitoring and alerting configured
- [ ] Runbooks documented for common issues

---

## 12. Handoff Notes

This specification is ready for implementation because:

1. **Clear scope**: MVP is well-defined; Phase 2 is deferred
2. **User-centered**: Built around real teacher pain points
3. **Technical approach**: Stack, data model, performance budgets specified
4. **Risk-aware**: Top risks identified with mitigations
5. **Measurable success**: Success metrics testable and validated

**For the implementation team:**
- This spec does NOT detail API endpoints or exact UI designs (those are design documents)
- This spec DOES provide the product requirements and acceptance criteria
- Design documents should follow; code should validate against this spec

**For the business team:**
- 16 weeks of engineering for MVP
- Recommend 5+ users in beta before public launch
- Plan for $200K-500K in infrastructure for Phase 1 scale
- Post-MVP: LMS integration critical for growth
```

---

## Part 4: Validate Your Specification

You've now written a **production-grade specification.** But is it actually good? **Ask your companion:**

```
I've written a comprehensive specification for RapidGrade,
a production grading system. Here's my spec:

[paste your complete spec]

Please review it as if you were a technical architect evaluating
whether this is ready for implementation planning.

Tell me:
1. What's clear and well-specified?
2. What's still vague or ambiguous?
3. What critical questions should I answer?
4. Is this specification complete enough for a team to plan from?
5. What did I miss that a production product needs?
```

Your companion will provide professional feedback like:

> "This is a solid specification. Strengths:
>
> - Clear problem statement grounded in real pain points
> - User personas are specific and realistic
> - Scope is well-bounded (MVP clear, Phase 2 deferred)
> - Technical constraints are realistic
> - Success metrics are measurable
>
> Areas to strengthen:
>
> - Security: No mention of data privacy, compliance (FERPA for education?)
> - Disaster recovery: What if database fails?
> - Onboarding: How do teachers learn to use it?
> - Pricing/licensing: How will this be monetized?
> - Internationalization: Is this US-only, or global?
>
> For implementation planning, I'd add:
>
> - Detailed API specification (this is product spec, not technical spec)
> - UI mockups or wireframes
> - Database schema
> - Deployment architecture diagram
>
> But as a product specification, this is strong. A team could take this
> and write a plan and start building."

---

## Part 5: Create Your Handoff Document

Now create a **Handoff Document** that an implementation team would use:

```markdown
# RapidGrade Implementation Handoff

## What Is This?

A complete product specification for RapidGrade, a grading system for educators.
This document is the contract between product and engineering: "This is what must be built."

## Key Decisions Made

1. **Scope**: MVP focuses on rubric + grading workflow; defers AI and LMS integration
2. **Users**: College professors and high school teachers (not K-5 initially)
3. **Tech Stack**: React/FastAPI/PostgreSQL (chosen for scalability)
4. **Deployment**: Cloud-based SaaS (AWS), not self-hosted
5. **Timeline**: 16 weeks to MVP

## What's NOT Included Here

- Detailed API specifications (engineering team will define)
- UI/UX mockups (design team will create)
- Database schema (data architect will detail)
- DevOps configuration (infrastructure team will design)

## What IS Included Here

- Product requirements (what must be built)
- User personas and use cases
- Success metrics (how we validate it works)
- Technical constraints and assumptions
- Risks and mitigations
- Implementation phases

## How to Use This

1. **For Planning**: Write detailed implementation plan based on features
2. **For Design**: Create UI/UX mockups that satisfy acceptance criteria
3. **For Engineering**: Build code that passes acceptance criteria tests
4. **For QA**: Validate against acceptance criteria checklist
5. **For Product**: Track progress against success metrics

## Questions for Implementers

Before you start, answer:

- How long to build Phase 1? (We estimate 16 weeks; you might estimate 12-20)
- What risks do you see? (Beyond what we listed)
- What dependencies do you need? (Databases? APIs? Third-party services?)
- What's your go/no-go criteria for launch?

## Success

This specification is implemented successfully when:

1. All acceptance criteria in Section 11 are met
2. All success metrics in Section 7 are validated
3. User testing with 5+ real teachers shows positive results
4. System passes performance and reliability tests
5. Team confidence is high that this solves the real problem

---

**Status**: Ready for implementation planning
**Next Step**: Engineering team writes detailed implementation plan
**Owner**: Product team
**Last Updated**: [Date]
```

---

## Part 6: Reflect on the Spec-Writing Process

This was different from Mini-Project 1. You didn't code anything. **Write a reflection:**

```markdown
## Reflection: Spec Writing vs. Implementation

### What Was Harder: Writing Spec or Building Code?

For me: Writing the spec was harder because...

- I had to think about scalability (not just MVP functionality)
- I had to anticipate user needs without directly coding
- I had to make architectural decisions without knowing implementation details
- I had to be specific enough for a team to plan from, but not prescriptive about how

### How Is This Spec Different From Mini-Project 1 (Calculator)?

The calculator spec was small (10 sections). This spec is production-grade (12+ sections).

Differences:

- Calculator spec: Focused on correctness (does math work?)
- RapidGrade spec: Focused on market fit (does this solve a real problem?)

- Calculator spec: Short term thinking (how to build fast)
- RapidGrade spec: Long term thinking (phases, scalability, risks)

- Calculator spec: Individual developer (one person builds it)
- RapidGrade spec: Team thinking (how does design team, eng team, QA team use this?)

### What's the Most Important Sentence in a Production Spec?

For me: The problem statement. If you don't have a clear problem statement,
everything downstream is guessing. Everything else (features, metrics, phases)
should flow from "what's the real problem we're solving?"

### How Has Your Understanding of Specification Changed?

Before Chapter 26: Specs are documents you write then put on a shelf.
After Chapter 26: Specs are contracts. They're how you communicate intent
to people (or AI) who will build your vision.
```

---

## Try With AI

You're going to complete this mini-project with professional-level rigor.

### Prompt 1: Understand the Real Problem

```
I'm going to build a production-grade specification for a grading system.
But before I write, I need to understand the real problem.

Tell me (don't propose software):
1. What's the real problem teachers face with grading?
2. What do they do now? (spreadsheets? LMS?)
3. What takes the most time?
4. What would actually help them?

Give me the problem, not solutions.
```

**Expected outcome**: You understand the real market problem. This grounds your spec in reality.

### Prompt 2: Coach Scope With Me

```
Based on the real problem you described, I want to scope a solution.

But I can't build everything, so help me answer:
1. Who's my first user? (Be specific: HS math? College essays?)
2. What's the absolute must-have feature set?
3. What can I reasonably defer to Phase 2?
4. What would make this different from existing tools?

Help me define realistic MVP scope.
```

**Expected outcome**: You have a clear MVP and Phase 2 vision.

### Prompt 3: Review My Production Spec

```
I've written a comprehensive specification for my grading system.

[paste your spec.md]

As a technical architect, review this:
1. Is this complete enough for a team to plan from?
2. What critical questions should I answer?
3. What am I missing?
4. Is scope realistic for Phase 1?
5. Would you invest in this product?
```

**Expected outcome**: You get professional feedback on spec quality.

### Prompt 4: Identify Next Steps

```
Based on my specification, what should the implementation team do next?

What comes after this spec?
- Detailed technical design?
- Architecture document?
- UI mockups?
- Implementation plan?

What's the right sequence?
```

**Expected outcome**: You understand the workflow from spec to implementation.

---

## Deliverable

Your finished mini-project should include:

1. **spec.md** — Production-grade specification (12+ sections)
2. **HANDOFF.md** — Document for implementation team
3. **REFLECTION.md** — Your learning from the project

This specification is **professional-quality**. It could be used to:

- Pitch investors
- Brief a development team
- Plan an implementation timeline
- Design the product

---

## What You've Accomplished

In Chapter 26, you've completed a full **specification-driven development education**:

**Lessons 1-7**: Learn the workflow

- Coach clarity → Specify → Refine → Plan → Decompose

**Mini-Project 1**: Build from a spec

- Spec → Code → Test → Validate

**Mini-Project 2**: Write a production spec

- Research → Define scope → Write spec → Handoff

You now understand something most developers never learn: **The quality of your thinking upstream (specification) determines the quality of your code downstream.**

This is the foundation of AI-native development. Clear specs → AI generates good code. Vague specs → AI generates confusing code.

You're ready to apply this to your own projects.
