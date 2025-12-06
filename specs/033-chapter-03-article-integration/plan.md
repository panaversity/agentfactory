# Implementation Plan: Chapter 3 Lesson 6 Article Integration

**Branch**: `033-chapter-03-article-integration` | **Date**: 2025-12-06 | **Spec**: `specs/033-chapter-03-article-integration/spec.md`

---

## Summary

Integrate "AI Is Quietly Creating Millionaires" (Entrepreneur.com, Simeon Ivanov, 2025) into Chapter 3 Lesson 6 "Three Requirements for Vertical Success" by adding a single 300-500 word subsection titled "Why Intelligence Is the New Competitive Asset." This subsection reinforces why domain expertise (fine-tuned models or vertical intelligence) is non-negotiable for building defensible solutions, connecting economic imperative to technical requirement. Article insights are reframed in educational language (removing entrepreneurial wealth-building narrative, retaining intelligence-as-moat concept). Update summary file to reflect new insight for student quick-reference.

**Learning Impact**: Students understand WHY all three requirements are necessary—not as arbitrary checklist, but as interconnected system where domain expertise determines defensibility, integrations enable access, and agentic solutions deliver value.

**Architectural Approach**: This is a TARGETED ADDITION, not a lesson redesign. The subsection inserts into existing structure (after Path 2 discussion, before Choosing Your Path section) without removing or restructuring existing content. Existing three requirements remain unchanged; this subsection adds the economic REASONING that ties them together.

---

## Technical Context

**Content Type**: Educational lesson subsection (Markdown)

**Chapter/Lesson**: Part 1, Chapter 3 "How to Make a Billion Dollars in the AI Era", Lesson 6 "Three Requirements for Vertical Success"

**Files to Modify**:
1. `book-source/docs/01-Introducing-AI-Driven-Development/03-billion-dollar-ai/06-three-requirements.md` — Add subsection
2. `book-source/docs/01-Introducing-AI-Driven-Development/03-billion-dollar-ai/06-three-requirements.summary.md` — Update summary

**Target Audience**: A1-A2 proficiency (beginner, no programming background)

**Duration**: Subsection adds 2 minutes to existing 12-minute lesson (total ~14 minutes)

**Pedagogical Layer**: Layer 1 Manual Foundation (direct explanation, mental models, no hidden scaffolding)

**Language Requirements**: A1-A2 appropriate (accessible vocabulary, explain jargon, no assumed technical knowledge)

**Proficiency Assessment**: This lesson remains "Understand" level (Bloom's L2), not "Apply" or higher. Students should recognize why domain expertise matters, not implement fine-tuning algorithms.

---

## Constitution Check

**Applied Framework**: Constitution v6.0.1 (Reasoning-Activated Edition)

### Principles Validation

**Principle 1: Specification Primacy (Intent Over Implementation)**
- ✓ Establishes WHAT (domain expertise as competitive moat) before HOW (fine-tuning vs vertical intelligence)
- ✓ Clarifies WHY all three requirements interconnect (answering the "so what?" question)
- ✓ Students understand defensibility concept before evaluating which path to domain expertise

**Principle 2: Progressive Complexity (Context-Appropriate Cognitive Load)**
- ✓ A2 limit: 5-7 new concepts. Current lesson: 5 concepts (fine-tuning, vertical intelligence, sub-agents/skills, integrations, agentic solutions)
- ✓ New subsection introduces ≤2 concepts: (1) intelligence as asset class, (2) defensibility through system complexity
- ✓ Total remains ≤7 concepts (within A2 limit)
- ✓ Chunking strategy: Group intelligence-as-moat and defensibility together as causal pair

**Principle 3: Factual Accuracy (Verification Over Assumption)**
- ✓ Article source: "AI Is Quietly Creating Millionaires", Entrepreneur.com, author Simeon Ivanov, 2025 (verifiable public source)
- ✓ Core claims testable: "Fine-tuned models take months to replicate" (supported by ML literature), "Vertical intelligence defensible because requires system-level knowledge" (observable in competitive analysis)
- ✓ Implementation step: Verify article source before finalizing content; cite directly with attribution

**Principle 4: Coherent Pedagogical Structure (Learning Progression)**
- ✓ Builds on Lesson 3 (super orchestrator economics) — students already understand value of accumulated intelligence
- ✓ Serves existing three-requirements learning objectives (no new objectives added)
- ✓ Positioned strategically: After explaining WHAT is required (both paths), students understand WHY both work (each builds defensible moat)
- ✓ Maintains lesson arc: Problem context → Two solutions → Why both valuable → Choosing path

**Principle 5: Intelligence Accumulation (Context-Rich Over Horizontal)**
- ✓ Draws from previous lesson (Lesson 3: super orchestrator economics, accumulation of intelligence)
- ✓ Connects to book's core thesis (reusable intelligence > reusable code)
- ✓ Contributes forward to Lesson 7 (students apply three-requirements framework when choosing vertical market)
- ✓ Constitutional source: Uses reasoning frameworks from Constitution (Principle 5 itself—Intelligence Accumulation)

**Principle 6: Anti-Convergence Variation (Distinctive Over Generic)**
- ✓ Lesson 5 (PPP Strategy) uses strategic narrative approach
- ✓ Lesson 6 teaches through: framework explanation + failure case study (OpenAI Study Mode) + expert insight + practice
- ✓ New subsection adds: ECONOMIC REASONING (from Entrepreneur article) to complement technical and strategic reasoning
- ✓ Modality variation: Adding economic perspective to lesson focused on technical/strategic requirements

**Principle 7: Minimal Sufficient Content (Essential Over Exhaustive)**
- ✓ Every section of new subsection maps to learning objective: "Understand why all three requirements are interdependent"
- ✓ No extraneous content (e.g., NOT adding details about specific AI techniques, NOT creating new exercises)
- ✓ Focused scope: One insight (intelligence as defensible asset) not multiple tangents
- ✓ Serves existing assessment method (students can explain why missing any requirement causes failure)

**Meta-Commentary Prohibition (Constitution v6.0.1)**
- ✓ New subsection is Layer 1 (Manual Foundation) — explains directly, no scaffolding exposure
- ✓ Forbidden patterns check:
  - NO "What to notice:" meta-commentary
  - NO "AI learns from you" exposition
  - NO "Three Roles framework" labels (this lesson doesn't use Three Roles—Layer 1 only)
- ✓ Grep validation required in QA phase

### Compliance Gate Results

**PASS** — All seven principles satisfied. Meta-commentary prohibition validated. Ready to proceed to Phase 0 research.

---

## Phase 0: Research & Article Content Extraction

### Research Objectives

1. **Verify Article Source**: Confirm "AI Is Quietly Creating Millionaires" by Simeon Ivanov, Entrepreneur.com, 2025 is publicly available
2. **Extract Core Thesis**: Identify the economic argument connecting AI to wealth creation and defensibility
3. **Identify Reframing Requirements**: Understand how entrepreneurial angle (millionaire-building) must shift to educational angle (defensibility-understanding)
4. **Validate Claims**: Check article's assertions against existing knowledge (e.g., "fine-tuned models take months to replicate" — supported by ML literature?)

### Research Methodology

**Step 1: Locate and Verify Article**
- Search: "AI Is Quietly Creating Millionaires" Simeon Ivanov Entrepreneur.com 2025
- Verify: Author, publication, date, public access
- Capture: URL and full text or key excerpts
- Action: If article not publicly available or claims cannot be verified, escalate to user for alternative source or fact-checking guidance

**Step 2: Extract Core Concepts**
- Read article fully
- Identify: Which passages support "intelligence as competitive asset"?
- Map: How does article explain defensibility? Which examples show system-level advantage?
- Document: Direct quotes vs paraphrasing opportunities
- Flag: Any claims requiring additional verification (e.g., specific statistics, named companies)

**Step 3: Analyze Tone & Language**
- Current article tone: Entrepreneurial, wealth-focused, aspirational ("build AI, become millionaire")
- Required reframing: Educational, learning-focused, evidence-based ("domain expertise creates defensible moat")
- Identify: Which insights translate directly, which require reinterpretation?
- Example reframing:
  - Article: "Millionaires are building AI systems faster than anyone else"
  - Reframe: "Companies with domain expertise build defensible AI solutions faster than generalists"

**Step 4: Validate Against Existing Lesson**
- Check article claims against lesson's existing discussion of fine-tuning vs vertical intelligence
- Verify: Does article support both paths as defensible?
- Flag conflicts: If article contradicts lesson's framework, escalate for resolution
- Confirm: Article enhances rather than contradicts existing content

### Deliverable from Phase 0

**Research Summary Document** (will be created during implementation):
- Article source verification (URL, access date)
- Core thesis extraction (2-3 sentence summary)
- Key quotes or paraphrasing opportunities (3-5 options)
- Tone analysis and reframing strategy
- Fact-check results (any claims requiring additional verification)
- Concept mapping (how article concepts align with lesson framework)

---

## Phase 1: Design & Content Outline

### Subsection Structure

**Location in Lesson**: After "Path 2: Vertical Reusable Intelligence" section (lines ~125), before "Choosing Your Path" section (lines ~127)

**Structural Placement** (in markdown):
```
## Requirement 1: Increase Domain Expertise...

### Path 1: Fine-Tuned Models
[existing content]

### Path 2: Vertical Reusable Intelligence...
[existing content]

### Why Intelligence Is the New Competitive Asset
[NEW SUBSECTION — will be inserted here]

### Choosing Your Path
[existing content]
```

**Subsection Word Count**: 300-500 words (target: 400 words to leave room for compression if needed)

**Subsection Outline** (conceptual structure, will be refined):

```
### Why Intelligence Is the New Competitive Asset

[Hook/Opening]: Why does this matter economically? (1-2 sentences)
- Connect to Lesson 3 context: Super orchestrators win through accumulated intelligence
- Establish stakes: Domain expertise determines defensibility

[Core Insight 1]: Intelligence replaces effort as the source of competitive advantage
- Explain: What's different about AI-driven competition vs traditional software?
- Evidence: (from article) Why are early adopters of domain-specific AI building defensible businesses?
- Implication: Your months of work building knowledge = competitor's barrier to entry

[Core Insight 2]: Both paths (fine-tuning AND vertical intelligence) are defensible BECAUSE they embody knowledge
- Explain: Why does it take competitors months to replicate your expertise?
- Evidence: Vertical intelligence = accumulated design decisions + domain-specific workflows + validation rules
- Evidence: Fine-tuned models = pattern recognition built into model itself
- Implication: The "secret sauce" is the accumulated intelligence, however encoded

[Core Insight 3]: This is why missing Requirement 1 is fatal
- Explain: Without domain expertise, you're selling generic AI wrapped differently
- Evidence: Competitor without moat can replicate your product in weeks (feature-for-feature)
- Implication: But competitor cannot replicate your months of domain knowledge—IF you encode it (fine-tuning or skills)

[Closing]: Why this matters for YOUR choice between Path 1 and Path 2
- Restate: Both create defensible moat because both embody knowledge
- Forward-look: Choose based on iteration speed and data availability, not defensibility (both equally defensible)
- Connection: This is the strategic insight that makes "all three requirements" an inseparable system
```

### Concept Inventory (New Concepts)

**Concepts introduced in new subsection**:
1. **Intelligence as competitive asset** (or "intelligence as defensive moat"): The idea that accumulated domain knowledge, whether encoded in fine-tuned models or vertical intelligence systems, creates defensibility that competitors cannot quickly replicate
2. **Defensibility through system complexity**: Why competitors must replicate not just individual components but the entire accumulated system (all design decisions, workflows, validations)

**Existing concepts reinforced** (not new, but deepened):
- Fine-tuned models (already explained in Path 1)
- Vertical intelligence/skills (already explained in Path 2)
- Competitive moat (referenced in lesson, now explained in economic context)

**Total concept count after addition**: 5 existing + 2 new = 7 concepts (at A2 ceiling—acceptable because addition serves core learning objective)

### Language & Tone Calibration

**Target Audience Adjustment**: A1-A2 (beginner, no programming background)

**Vocabulary Guardrails**:
- ✓ Explain "fine-tuned model" if used (not assumed knowledge)
- ✓ Define "competitive moat" plainly ("advantage that competitors struggle to replicate")
- ✓ Avoid jargon without explanation (e.g., "feature parity," "differentiation," "proprietary algorithms")
- ✓ Use concrete examples from lesson (healthcare, finance) over abstract concepts

**Sentence Structure**:
- Keep sentences short and direct
- Avoid nested complexity ("If X, then Y, but only when Z...")
- Use parallel structure for comparisons ("Path 1 requires X; Path 2 requires Y")

**Voice**:
- Direct address ("You're building...") vs passive ("A system is built...")
- Active explanation ("Intelligence creates defensibility because competitors must replicate your entire system")
- Concrete examples over abstract principles

### Connection to Lesson 3 (Super Orchestrators)

**Explicit Cross-Reference Required** in new subsection:

Lesson 3 introduced: "Instagram had 13 people who could generate billion-dollar value by orchestrating AI well. That orchestration skill—accumulated intelligence about how to coordinate systems—is what made them powerful."

**New subsection must say something like**: "This is why super orchestrators from Lesson 3 were powerful—their accumulated intelligence about coordinating AI systems creates defensibility. You're building the same advantage when you invest months in domain expertise. Your expertise, like Instagram's orchestration knowledge, becomes the moat."

### Summary File Update Strategy

**Current Summary Structure** (06-three-requirements.summary.md):
- Core Concept
- Key Mental Models (3-4 models)
- Critical Patterns (4-5 patterns)
- Common Mistakes (3-4 items)
- Connections (Builds on / Leads to)

**Addition Points**:
1. **Core Concept**: Add sentence about intelligence as moat ("...Intelligence as competitive asset—both fine-tuned models and vertical intelligence are defensible because they embody months of accumulated domain knowledge.")

2. **Key Mental Models**: Add new model: "**Intelligence as Defensive Moat**: Accumulated domain knowledge (whether fine-tuned into models or structured as skills/workflows) creates defensibility that competitors cannot quickly replicate"

3. **Critical Patterns**: Add pattern about defensibility through complexity ("Defensibility comes from system-level knowledge, not individual components—skill libraries are equally defensible as fine-tuned models")

4. **Connections**: Ensure "Builds on Lesson 3 (super orchestrator economics)" is explicit; add "Concept of accumulated intelligence as defensibility foundation" if not present

---

## Phase 2: Implementation Approach

### Task Sequence (Detailed Steps)

**Task 1: Research & Verification (1-2 hours)**
- Locate article "AI Is Quietly Creating Millionaires" (Entrepreneur.com, Simeon Ivanov)
- Verify access and public availability
- Read full text
- Extract core thesis and key quotes
- Document findings in research working file
- Output: Research findings (to inform content writing)

**Task 2: Concept Development (2-3 hours)**
- Using research findings, develop 3-4 different approaches to explain "intelligence as competitive asset"
- Write 3 alternative subsection drafts (different emphasis/examples)
- Evaluate each against success criteria:
  - Cognitive load (≤2 new concepts)
  - Language (A1-A2 appropriate)
  - Connection to Lesson 3 (explicit cross-reference)
  - Constitutional compliance (no meta-commentary)
- Select strongest draft
- Output: Draft subsection content

**Task 3: Integration & Refinement (1-2 hours)**
- Insert draft subsection into 06-three-requirements.md at specified location
- Check surrounding context (does it follow naturally from Path 2? Does it lead smoothly to Choosing Your Path?)
- Refine transitions (add connective language if needed)
- Word count check (target 300-500, adjust if necessary)
- Grammar and clarity review
- Output: Integrated lesson file with subsection

**Task 4: Summary Update (30-45 minutes)**
- Update 06-three-requirements.summary.md
- Add "intelligence as moat" concept to Core Concept section
- Add new Key Mental Model about defensibility through complexity
- Add critical pattern about system-level knowledge
- Ensure consistency between lesson subsection and summary language
- Output: Updated summary file

**Task 5: Validation (1-2 hours)**
- Constitutional compliance check:
  - Grep for forbidden meta-commentary patterns
  - Layer 1 approach confirmation (direct explanation, no hidden pedagogy)
  - Specification primacy check (explains WHAT and WHY before HOW)
- Cognitive load audit: Count new concepts, verify ≤2 additions, total ≤7
- Language audit: Review for jargon without explanation, check sentence structure
- Cross-reference verification: Does subsection explicitly reference Lesson 3?
- Article authenticity check: Are article insights genuinely reflected (not tokenized)?
- Output: Validation checklist results

**Task 6: Quality Assurance (30-45 minutes)**
- Read full lesson flow (Requirement 1 intro → Path 1 → Path 2 → NEW SUBSECTION → Choosing Path)
- Check for repetition (does new subsection duplicate existing content?)
- Verify timing (~2 minute addition to 12-minute lesson = 14 min total)
- Proofread for typos, formatting consistency, markdown syntax
- Compare summary against lesson for consistency
- Output: Final version ready for review

### Implementation Criteria (Definition of Done)

For each task, "done" means:

**Task 1 (Research)**:
- Article source located and verified
- Full text read and understood
- Core thesis (2-3 sentences) documented
- Key quotes (3-5 options) captured with page numbers
- Tone analysis (how to reframe for educational audience) written
- All findings in working research document

**Task 2 (Concept)**:
- 3 draft approaches written (not polished, but conceptually distinct)
- Each draft evaluated against 4 criteria (cognitive load, language, connection, constitutional)
- Strongest draft selected with documented rationale
- Draft reviewed for coherence and clarity

**Task 3 (Integration)**:
- Subsection inserted at correct location in markdown file
- Surrounding context verified (flows from Path 2, leads to Choosing Path)
- Word count: 300-500 words ± 10%
- Grammar and spelling checked (no obvious errors)

**Task 4 (Summary)**:
- Core Concept updated with intelligence-as-moat reference
- New Key Mental Model added (Defensibility through complexity)
- Critical Pattern added (system-level knowledge)
- Language matches lesson subsection language (consistency check)
- All changes marked clearly for review

**Task 5 (Validation)**:
- Grep checks passed (zero forbidden meta-commentary matches)
- Cognitive load audit: ≤2 new concepts confirmed, total ≤7 confirmed
- Language audit: All technical terms explained or familiar from lesson context
- Cross-reference check: Lesson 3 explicitly mentioned with short context
- Article authenticity: Core insights present, not just token mentions
- All checks documented in validation report

**Task 6 (QA)**:
- Full lesson read top-to-bottom for flow and coherence
- No repetition with existing content detected
- Timing estimate valid (~2 min for subsection)
- No formatting or markdown errors
- Summary and lesson language consistent
- Ready for human review

### Files Involved

**Files to Create**: None (this is addition to existing files)

**Files to Modify**:
1. `book-source/docs/01-Introducing-AI-Driven-Development/03-billion-dollar-ai/06-three-requirements.md`
   - Location: After "### Path 2: Vertical Reusable Intelligence..." section (line ~125)
   - Action: INSERT new subsection "### Why Intelligence Is the New Competitive Asset"
   - Size: ~400 words

2. `book-source/docs/01-Introducing-AI-Driven-Development/03-billion-dollar-ai/06-three-requirements.summary.md`
   - Location: "### Core Concept" section, "### Key Mental Models" section, "### Critical Patterns" section
   - Action: ADD references to intelligence-as-moat / defensibility-through-complexity
   - Size: 3-4 lines of additions across sections

**Files NOT Modified** (no need to touch):
- Lesson 5, Lesson 7, or other chapter files (no structural dependencies)
- Quiz file (08_chapter_03_quiz.md) — no changes needed; assessment already covers three requirements
- Learning objectives (embedded in 06-three-requirements.md metadata) — no new objectives added

---

## Quality Validation Checkpoints

### Constitutional Compliance Validation

**Principle 1 (Specification Primacy)** ✓
- [ ] Subsection explains WHY before HOW
- [ ] Establishes defensibility concept before comparing paths
- [ ] Answers "so what?" question about domain expertise

**Principle 2 (Progressive Complexity)** ✓
- [ ] New concepts count: ≤2 (validated via audit)
- [ ] Total concepts: ≤7 (validated via audit)
- [ ] Language appropriate for A1-A2 (vocabulary audit passed)

**Principle 3 (Factual Accuracy)** ✓
- [ ] Article source verified (URL, access date documented)
- [ ] Core claims cited with attribution
- [ ] No hallucinations or unsupported assertions

**Principle 4 (Coherent Structure)** ✓
- [ ] Builds on Lesson 3 (explicit cross-reference present)
- [ ] Serves existing learning objectives
- [ ] Positioned strategically in lesson arc

**Principle 5 (Intelligence Accumulation)** ✓
- [ ] References prior lesson (Lesson 3 super orchestrators)
- [ ] Contributes to book's thesis (reusable intelligence)
- [ ] Supports future lesson (Lesson 7 vertical choice)

**Principle 6 (Anti-Convergence)** ✓
- [ ] Adds economic reasoning to complement technical/strategic
- [ ] Modality different from Lesson 5 (not purely strategic narrative)

**Principle 7 (Minimal Sufficient)** ✓
- [ ] Every section maps to learning objective
- [ ] No extraneous technical details
- [ ] Focused scope (one insight, not multiple tangents)

**Meta-Commentary Prohibition (v6.0.1)** ✓
- [ ] Grep check: `"What to notice|What to expect|AI.*teach|AI.*learn|teach.*AI|AI as|AI now knows|AI adapted"` — ZERO matches expected
- [ ] Layer 1 approach confirmed (direct explanation, no scaffolding exposure)

### Cognitive Load Audit

**Existing Concepts (from lesson)**: 5 total
1. Fine-tuning
2. Vertical intelligence
3. Sub-agents/skills
4. Deep integrations
5. Agentic solutions

**New Concepts (in subsection)**:
- [ ] Concept 1: Intelligence as competitive asset (or "intelligence as defensive moat")
- [ ] Concept 2: Defensibility through system complexity

**Total After Addition**: 7 concepts (= A2 ceiling of 5-7)

**Validation**: Count = 7 ≤ A2 limit ✓

### Language Audit

**A1-A2 Language Check**:
- [ ] No unexplained jargon (if used, explained in context or previously in lesson)
- [ ] Sentence length: average <20 words
- [ ] Concrete examples provided (healthcare, finance, etc.)
- [ ] Active voice preferred over passive
- [ ] No assumption of programming background

**Specific Terms Check**:
- [ ] "Fine-tuning" — already explained in Path 1 section, reference back if needed
- [ ] "Competitive moat" — define in plain language ("advantage competitors struggle to replicate")
- [ ] "Defensibility" — explain as "difficulty/cost for competitors to replicate"
- [ ] "Vertical intelligence" — already explained in Path 2 section, reference if used
- [ ] "Pattern recognition" — explain in context

### Connection to Prior Learning

**Lesson 3 Reference**:
- [ ] Subsection explicitly mentions "super orchestrators from Lesson 3" or "Lesson 3 showed..."
- [ ] Connection stated clearly: How does this subsection deepen Lesson 3's insight?
- [ ] Cross-reference sufficient for students who read Lesson 3 AND new readers

**Summary Consistency**:
- [ ] Summary intelligence-as-moat language mirrors subsection language
- [ ] Terminology consistent between lesson and summary
- [ ] Summary provides quick reference for subsection insight

### Article Authenticity Check

**Core Thesis Integration**:
- [ ] Article's "intelligence as wealth driver" concept present
- [ ] Article's "defensibility through accumulated systems" present
- [ ] Article's "early adopter advantage" mentioned (if applicable)
- [ ] Integration is substantive (not just token mentions)

**Tone Reframing**:
- [ ] Entrepreneurial tone replaced with educational tone
- [ ] Economic insight preserved (wealth-building frame → defensibility-understanding frame)
- [ ] No loss of core message in translation

---

## Success Metrics Summary

| Metric | Target | Validation Method |
|--------|--------|-------------------|
| Cognitive Load | ≤2 new concepts, total ≤7 | Concept count audit |
| Word Count | 300-500 words | Word processor check |
| Language | A1-A2 appropriate | Vocabulary audit, readability |
| Constitutional | All 7 principles satisfied | Principle checklist |
| Meta-Commentary | Zero forbidden patterns | Grep validation |
| Cross-Reference | Explicit Lesson 3 mention | Content review |
| Summary Update | Consistent with subsection | Language comparison |
| Article Integration | Authentic, not tokenized | Concept alignment review |

---

## Risk Mitigation

### Risk: Article Source Unavailable

**Probability**: Low (Entrepreneur.com is major publication, article is recent)
**Impact**: High (cannot integrate if source inaccessible)
**Mitigation**: Research phase includes verification step. If article inaccessible, escalate to user before Task 2 proceeds.

### Risk: Article Claims Conflict with Lesson Framework

**Probability**: Low (article was suggested based on alignment with lesson theme)
**Impact**: Medium (creates contradiction students must resolve)
**Mitigation**: Phase 1 research includes conflict detection. If contradiction found, escalate for resolution (e.g., reframe article claim or adjust lesson language).

### Risk: Subsection Exceeds Cognitive Load Limit

**Probability**: Low (framework constrains to ≤2 new concepts)
**Impact**: Medium (violates A2 proficiency constraint)
**Mitigation**: Task 2 includes word count and concept audits. If limit exceeded, compress or split into sub-sections (though this risks violating "targeted addition" requirement).

### Risk: Subsection Violates Layer 1 (Manual Foundation) Approach

**Probability**: Low (spec and plan emphasize maintaining Layer 1)
**Impact**: High (violates pedagogical framework)
**Mitigation**: Task 5 includes explicit Layer 1 validation. Any "Try With AI" or collaborative prompts inserted will be flagged and removed.

### Risk: Summary Update Creates Inconsistency

**Probability**: Low (same person implementing both tasks)
**Impact**: Low (confusing for students, but not blocking)
**Mitigation**: Task 4 includes consistency cross-check. Language in summary must match lesson subsection.

---

## Implementation Timeline

**Estimated Total Duration**: 5-8 hours (including research, writing, validation, QA)

**Phased Timeline**:
- Phase 0 (Research): 1-2 hours → Research findings document
- Phase 1 (Design): 2-3 hours → Draft subsection
- Phase 2 (Implementation): 1-2 hours → Integrated content + summary update
- Validation & QA: 1.5-2 hours → Validation report + final version

**Assuming 2-3 hour per day work sessions**: 2-3 days to completion

---

## Acceptance Criteria (Ready for Implementation)

Plan is READY when:
- [ ] All phases defined (Research → Design → Implementation → Validation)
- [ ] Tasks sequenced with clear deliverables
- [ ] Success metrics quantified (not subjective)
- [ ] Constitutional compliance explicitly mapped
- [ ] Risk mitigation documented
- [ ] Files identified (what to modify, what not to touch)
- [ ] Timeline estimated

**Current Status**: READY ✓

---

## Next Steps

1. **Approve Spec**: Confirm spec.md is complete and accurately describes feature
2. **Approve Plan**: Confirm plan.md provides clear implementation path
3. **User Confirmation**: Get user sign-off on scope (targeted addition, not redesign) and success criteria
4. **Proceed to Tasks**: Once approved, generate `tasks.md` with granular implementation steps

**Estimated Time to Implementation Approval**: 30 minutes (assuming no clarifications needed)

