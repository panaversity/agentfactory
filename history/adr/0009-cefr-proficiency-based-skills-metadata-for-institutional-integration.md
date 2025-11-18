# ADR-0009: CEFR Proficiency-Based Skills Metadata for Institutional Integration

> **Scope**: Pedagogical infrastructure for tagging lessons with international standards-based proficiency levels (CEFR A1-C2, Bloom's Taxonomy, DigComp 2.1) to enable competency-based assessment, portable credentials, and institutional accreditation alignment.

- **Status:** Accepted
- **Date:** 2025-11-08
- **Feature:** Chapter 18 - Lists, Tuples, and Dictionary (first implementation)
- **Context:** Part 4 Python Fundamentals (Chapters 12-29) - Establishing skills metadata infrastructure for entire book

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: ✅ YES - Affects how all chapters document skills; enables institutional partnerships, accreditation, digital credentials
     2) Alternatives: ✅ YES - Considered ad-hoc skill labels, custom levels, no metadata (status quo)
     3) Scope: ✅ YES - Cross-cutting pedagogical infrastructure affecting all 57 chapters and external integrations
-->

## Decision

**Adopt CEFR (Common European Framework of Reference) proficiency levels as the primary skill proficiency system**, augmented with Bloom's Taxonomy cognitive levels and DigComp 2.1 digital competence categories, for documenting learner competencies across all book chapters.

### Core Framework Components:

#### 1. CEFR Proficiency Levels (Primary)
**6 levels spanning basic to mastery**:
- **A1 (Beginner)**: Can recognize and use familiar everyday expressions; very basic understanding
- **A2 (Elementary)**: Can understand frequently used expressions; simple tasks in familiar situations
- **B1 (Intermediate)**: Can deal with most situations; produce simple connected text on familiar topics
- **B2 (Upper Intermediate)**: Can interact with fluency; understand complex technical text
- **C1 (Advanced)**: Can express ideas fluently; use language flexibly for social, academic, professional purposes
- **C2 (Mastery)**: Can understand virtually everything; express self spontaneously and precisely

**Chapter 18 Usage**: A1 → A2 → A2-B1 → B1 progression across 11 lessons

#### 2. Bloom's Taxonomy (Cognitive Complexity)
**6 cognitive levels for assessment design**:
- Remember → Understand → Apply → Analyze → Evaluate → Create

**Alignment with CEFR**:
- A1 → Remember/Understand
- A2 → Understand/Apply
- B1 → Apply/Analyze
- B2 → Analyze/Evaluate
- C1/C2 → Evaluate/Create

#### 3. DigComp 2.1 (Digital Competence Categories)
**5 competence areas** (subset relevant to software development):
- Information and Data Literacy
- Communication and Collaboration
- Digital Content Creation
- Safety
- Problem Solving

### Implementation in Chapter Plan:

Each lesson documents:
```yaml
Skills Taught:
- Skill Name — CEFR Level — Category — Measurable Outcome
Example:
- List Comprehension Syntax — B1 — Technical — Student can write [expr for item in iterable if condition] with correct semantics
```

### Validation Requirements:

**Five Coherence Tests** (applied in plan.md):
1. **Uniqueness**: No duplicate skills (semantic clarity)
2. **Naming Convention**: Consistent verb semantics (Recognizing, Understanding, Creating, Applying, Analyzing)
3. **Proficiency Progression**: Non-regressing levels (no A2 → A1 backward steps)
4. **Prerequisites**: Dependencies met before dependent skills appear
5. **Connectivity**: Skills connected across vertical (deepening) and horizontal (integration) tracks

### Metadata Storage:

- **Plan.md**: Skills metadata with coherence validation
- **Lesson YAML Frontmatter** (hidden from students): CEFR level, Bloom's level, skills taught
- **Master Skills Registry** (future): Central database for cross-chapter skill tracking

## Consequences

### Positive

1. **Internationally Recognized Standards**: CEFR used in 40+ countries, recognized by ESCO (European Skills framework), validated across 40+ languages over 40+ years

2. **Competency-Based Assessment**: Focus shifts from "passed test" to "can DO X at proficiency level Y" (employer-relevant)

3. **Portable Credentials**: Students can claim "B1 Python Collections" on LinkedIn/resume with internationally understood meaning

4. **Institutional Accreditation Alignment**: Universities, bootcamps, training providers can map content to local/national standards (ESCO, DigComp, NQF)

5. **Differentiation Design**: Clear proficiency targets enable:
   - Extension challenges for B1+ students
   - Remedial support for A1 students
   - Adaptive learning paths

6. **Cognitive Load Validation**: CEFR levels correlate with concept limits (A1: max 5, A2: max 7, B1: max 10), enabling quantitative validation

7. **Assessment Quality**: Bloom's taxonomy ensures "Try With AI" prompts progress from Remember → Analyze/Create (not all "Apply")

8. **Research-Grounded**: 70+ years of Bloom's research, 40+ years of CEFR research, 10+ years of DigComp research

9. **Cross-Language Consistency**: CEFR enables bilingual teaching (Python Part 4, TypeScript Part 9) with consistent proficiency expectations

10. **Digital Credential Readiness**: Metadata structured for Open Badges, Credly, blockchain credentials

### Negative

1. **Metadata Overhead**: content-implementer must tag every skill with CEFR + Bloom's + category (adds 10-15 minutes per lesson)

2. **Learning Curve**: Authors unfamiliar with CEFR need training (mitigated: skills-proficiency-mapper skill provides guidance)

3. **Granularity Debates**: A2 vs A2-B1 vs B1 proficiency boundaries are somewhat subjective (mitigated: coherence validation tests)

4. **Maintenance Burden**: If CEFR framework updates (last update 2001 → 2018 expansion), content must be re-validated

5. **Student Confusion Risk**: If exposed prematurely, "A1/A2/B1" labels may confuse students expecting traditional grades

6. **Not Universally Adopted**: Some employers unfamiliar with CEFR (more common in Europe/Asia than North America)

7. **Performance vs Proficiency**: CEFR measures "can do" proficiency, not speed/efficiency (separate performance metrics needed)

## Alternatives Considered

### Alternative A: Custom Skill Levels (Beginner/Intermediate/Advanced)
**Structure**:
- 3 levels: Beginner, Intermediate, Advanced
- No international standard alignment

**Why Rejected**:
- Not portable (what does "Intermediate Python" mean to an employer in Germany vs India?)
- No institutional accreditation pathway
- Arbitrary boundaries (when does Beginner → Intermediate?)
- Misses cognitive complexity dimension (Bloom's)

### Alternative B: Bloom's Taxonomy Only
**Structure**:
- Use 6 Bloom's levels (Remember → Create) without CEFR
- Focus on cognitive complexity, not proficiency

**Why Rejected**:
- Bloom's alone doesn't capture proficiency (can Remember advanced concepts vs Apply basic concepts)
- No institutional recognition (CEFR has 40+ years of adoption)
- Doesn't address differentiation (Bloom's doesn't prescribe remedial vs extension paths)

### Alternative C: No Metadata (Status Quo)
**Structure**:
- Lessons have no explicit proficiency tags
- Implicit difficulty through chapter ordering

**Why Rejected**:
- No competency-based assessment (can't prove "student achieved B1 Collections")
- No institutional partnerships (can't map content to accreditation standards)
- No differentiation (can't identify A1 vs B1 students for adaptive paths)
- Misses opportunity for digital credentials (Open Badges require metadata)

### Alternative D: DigComp 2.1 Only
**Structure**:
- Use EU Digital Competence Framework (8 proficiency levels)
- Focus on digital literacy, not language proficiency

**Why Rejected**:
- DigComp designed for digital literacy (using software), not programming proficiency (creating software)
- Less internationally recognized than CEFR outside Europe
- Doesn't integrate with language learning research (CEFR does)

## References

- Feature Spec: `/specs/001-part-4-chapter-18/spec.md`
- Implementation Plan: `/specs/001-part-4-chapter-18/plan.md` (Section: Skills Proficiency Metadata with 5 coherence tests)
- Related ADRs:
  - ADR-0008: 11-Lesson Collections Structure (enabled by proficiency metadata)
  - ADR-0006: 5-Lesson Operator Separation (precedent for cognitive load limits)
- Constitution: `.specify/memory/constitution.md` (Principle 12: Cognitive Load Management)
- Skills-Proficiency-Mapper: `.claude/skills/skills-proficiency-mapper/` (tool for applying CEFR validation)
- Research Foundation:
  - CEFR: Council of Europe (2001, expanded 2018) — 40+ years of language proficiency research
  - Bloom's Taxonomy: Bloom et al. (1956), revised Anderson & Krathwohl (2001) — 70+ years of cognitive science
  - DigComp 2.1: EU Joint Research Centre (2017, updated 2022) — Digital competence framework for EU
  - ESCO (European Skills/Competences): https://esco.ec.europa.eu/en/classification/skill_main
- Industry Alignment:
  - LinkedIn Skills: Increasingly recognizes CEFR-level language proficiency
  - Open Badges Specification v3.0: Supports CEFR metadata for competency credentials
  - ISO 21001:2018 Educational Organizations Management: References CEFR for proficiency documentation
