# ADR-002: Part 5 Progressive Proficiency Scaffolding (CEFR A2→B1→B2)

> **Scope**: Document cognitive load management and proficiency progression strategy for Part 5 Spec-Kit Plus Methodology. This decision clusters proficiency targets, cognitive load limits, and skills mapping to international standards.

- **Status:** Accepted
- **Date:** 2025-11-05
- **Feature:** Part 5 Spec-Kit Plus Methodology (Chapters 30-32)
- **Context:** Learners range from aspiring developers (first SDD experience) to professional developers (adopting AI-native workflows). Must balance accessibility with professional depth. Need measurable progression that supports institutional accreditation.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: YES - Determines assessment strategy, cognitive load per lesson, skills mapping for all Part 5 chapters
     2) Alternatives: YES - Multiple proficiency frameworks evaluated (ad-hoc, Bloom's only, CEFR, custom levels)
     3) Scope: YES - Cross-cutting concern affecting lesson design, exercise complexity, assessment rubrics across Part 5
-->

## Decision

**Adopt CEFR-Aligned Progressive Scaffolding**: Map all Part 5 lessons to international Common European Framework of Reference (CEFR) proficiency levels with explicit cognitive load management:

**Proficiency Progression**:
- **Lessons 1-3** (Foundation): A2 (Basic Application) - Max 5 new concepts per lesson
- **Lessons 4-6** (Tool-Assisted): B1 (Intermediate Application) - Max 7 new concepts per lesson
- **Lesson 7** (Integration): B1-B2 (Intermediate to Advanced) - Max 10 new concepts
- **Capstone**: B2 (Advanced) - Design decisions, synthesis, evaluation

**Bloom's Taxonomy Alignment**:
- A2 Lessons: Remember + Understand + Apply (guided contexts)
- B1 Lessons: Apply + Analyze (independent application)
- B2 Lesson/Capstone: Apply + Analyze + Evaluate + Create (design and synthesis)

**Skills Proficiency Metadata**: Document proficiency level for each skill taught:
- Format: `[Skill Name] — [CEFR Level] — [Category] — [Measurable at this level]`
- Example: "Specification Writing — B1 — Technical — Student can write complete spec without template for real-world problem"
- Enable institutional integration (ESCO, DigComp 2.1, local competency frameworks)

**Cognitive Load Validation**: Every lesson validates concept count against tier limits before content creation.

## Consequences

### Positive

- **International Standards Alignment**: CEFR levels recognized globally; enables institutional accreditation
- **Predictable Progression**: Students know what proficiency level they're working toward
- **Cognitive Load Protection**: Hard limits prevent overwhelming learners (especially A2: max 5 concepts)
- **Competency-Based Assessment**: What students CAN DO, not just test scores
- **Portable Credentials**: A1/A2/B1 levels transfer across institutions/countries
- **Differentiation Design**: Extension exercises for B1+ students, remedial for A1
- **Research-Grounded**: CEFR (40+ years language learning research), Bloom's (70+ years cognitive science), DigComp (2022 EU digital competence framework)
- **Skills-Based Hiring**: Graduates can demonstrate "B1 Specification Writing" on resumes

### Negative

- **Framework Overhead**: Requires proficiency mapping for every skill taught
- **Learning Curve**: Instructors must understand CEFR levels and Bloom's taxonomy
- **Rigidity**: Hard cognitive load limits may constrain complex topics
- **Mismatch Risk**: CEFR designed for language learning; mapping to technical skills requires interpretation
- **Assessment Burden**: Rubrics must explicitly assess proficiency levels (not just "good/needs work")
- **Student Confusion**: Some learners unfamiliar with CEFR levels need orientation

## Alternatives Considered

### Alternative A: Ad-Hoc Complexity Management
**Structure**: Instructor intuition guides lesson complexity; no formal proficiency framework.

**Why Rejected**:
- No measurable progression; students don't know what level they've achieved
- No institutional accreditation support
- Inconsistent cognitive load across lessons (some overwhelm, some bore)
- Can't demonstrate competency to employers ("I completed a chapter" vs. "I achieved B1 proficiency")

### Alternative B: Bloom's Taxonomy Only
**Structure**: Use Bloom's cognitive levels (Remember → Create) without CEFR proficiency mapping.

**Why Rejected**:
- Bloom's describes cognitive complexity but not skill proficiency
- No portable credentials (Bloom's levels don't transfer to other institutions)
- No cognitive load management (Bloom's doesn't specify concept count limits)
- Missing international standards alignment (Bloom's is US-centric)

### Alternative C: Custom Proficiency Levels (Beginner → Expert)
**Structure**: Define custom 5-level progression: Novice → Beginner → Intermediate → Advanced → Expert.

**Why Rejected**:
- Not recognized by institutions (no transfer value)
- No research foundation (ad-hoc definitions)
- No cognitive load research backing concept count limits
- Can't map to DigComp, ESCO, or other competency frameworks
- Requires extensive documentation to explain custom levels

### Alternative D: Uniform B1 Proficiency
**Structure**: All Part 5 lessons target B1 (Independent User) proficiency uniformly.

**Why Rejected**:
- No progression; beginners overwhelmed, advanced learners bored
- Violates graduated complexity principle (Constitution Principle 12)
- Can't differentiate lesson complexity (all lessons feel same difficulty)
- Misses opportunity for capstone to demonstrate B2 mastery

## References

- Feature Spec: `specs/010-chapter-31-redesign/spec.md` (Lines 68-106: Proficiency Targets)
- Implementation Plan: `specs/010-chapter-31-redesign/plan.md` (Lines 1437-1520: Skills Proficiency Mapping)
- Related ADRs: ADR-001 (Workflow Isomorphism), ADR-003 (Human Control Pattern)
- Constitution: `.specify/memory/constitution.md` (Principle 12: Cognitive Load Management)
- Skills Proficiency Mapper: `.claude/skills/skills-proficiency-mapper/`
- Research Foundation:
  - CEFR: 40+ years language learning proficiency research, validated across 40+ languages
  - Bloom's Taxonomy: 70+ years cognitive complexity research (1956 original, 2001 revision)
  - DigComp 2.1: Latest (2022) EU digital competence framework

## Why This Won

1. **International Recognition**: CEFR levels recognized by 40+ countries; enables institutional partnerships
2. **Research-Grounded**: 40+ years CEFR research + 70+ years Bloom's research; not ad-hoc
3. **Cognitive Load Protection**: Hard limits (A2: 5, B1: 7, B2: 10 concepts) prevent overload
4. **Competency-Based**: "Student can write complete spec" (measurable) vs. "knows specifications" (vague)
5. **Portable Credentials**: "B1 Specification Writing" transfers to employers/institutions
6. **Differentiation Support**: Extension for B1+, remedial for A1 students
7. **DigComp Alignment**: Maps to EU Digital Competence Framework (institutional accreditation)
8. **Skills-Based Hiring**: Graduates demonstrate specific competencies with recognized proficiency levels

## Implementation Notes

**Skills Metadata Format** (added to all lessons):
```markdown
#### Skills Taught

1. **Specification Writing** — B1 — Technical
   - Measurable at this level: Student can write complete spec without template for real-world problem
   - Category: Technical (core SDD skill)
   - DigComp: Content Creation, Communication

2. **AI Feedback Interpretation** — B1 — Soft
   - Measurable at this level: Student reads AI feedback; understands gap rationale; applies meaningfully
   - Category: Soft (critical thinking + collaboration)
   - DigComp: Communication & Problem-Solving
```

**Cognitive Load Validation** (before content creation):
- A2 Lessons: Count new concepts; reject if >5
- B1 Lessons: Count new concepts; reject if >7
- B2 Lessons: Count new concepts; reject if >10

**Proficiency Progression Validation** (across lessons):
- Does chapter follow A2→B1→B2 progression?
- Are prerequisites from earlier chapters satisfied (earlier skills at lower proficiency)?
- Does proficiency increase match learning objectives?

## Decision History

- **2025-11-05**: ADR created documenting proficiency scaffolding for Part 5
- **2025-11-03**: Initial proficiency targets defined in Chapter 31 spec
- **2025-10-31**: Constitution v3.0.0 established Principle 12 (Cognitive Load Management)
