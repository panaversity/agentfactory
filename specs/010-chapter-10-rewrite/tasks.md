# Chapter 10 Rewrite Tasks: Implementation Checklist

**Specification**: specs/010-chapter-10-rewrite/spec.md (APPROVED)
**Plan**: specs/010-chapter-10-rewrite/plan.md (APPROVED)
**Created**: 2025-11-18

---

## Task Overview

**Total Tasks**: 10 (8 lessons + 1 README + 1 validation)

**Implementation Strategy**: Sequential (L1 → L8 → README → Validation)

**Deliverables**:
- 8 lesson markdown files
- 1 chapter README.md
- Validation report

---

## TASK 1: Implement Lesson 1 - Prompts as Specifications

**File**: `book-source/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/01-prompts-as-specifications.md`

**Stage**: 1 (Manual Foundation)
**Duration**: 30 minutes
**Concepts**: 5

### Deliverables

- [x] Lesson markdown file with YAML frontmatter
- [x] 5 prompt examples (3 specification-quality, 2 vague)
- [x] Jake Heller framework integration (60% → 97% with citation)
- [x] Comparison chart exercise (Specification vs Vague prompts)

### Content Requirements

**Required Sections**:
1. Introduction (Why prompts are specifications)
2. Jake Heller Framework explanation (with YouTube timestamp citation)
3. Specification-first principle
4. 5 example prompts with analysis
5. Manual exercise (NO "Try With AI" - Stage 1)

**Prohibited Content**:
- ❌ NO context window explanations
- ❌ NO AI tool usage sections
- ❌ NO Python code examples
- ❌ NO meta-commentary ("You're learning Stage 1")

### Validation Criteria

- [ ] 5 concepts present (prompts as specs, WHAT vs HOW, spec-first, iteration framework, quality=output quality)
- [ ] Jake Heller quote cited with [20:03] timestamp
- [ ] ZERO AI collaboration sections (Stage 1 = manual only)
- [ ] Practice vehicle = markdown analysis (appropriate for Part 3)
- [ ] Cognitive load = 5 concepts (within A2 limit of 5-7)
- [ ] NO final sections beyond manual exercise (no "Try With AI")

### Anti-Pattern Checks

- [ ] ✅ No context engineering content
- [ ] ✅ No Python examples
- [ ] ✅ No unverified statistics
- [ ] ✅ No meta-commentary exposure
- [ ] ✅ Teaching modality = specification-first (not direct lecture)

---

## TASK 2: Implement Lesson 2 - Anatomy of Effective Prompts

**File**: `book-source/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/02-anatomy-effective-prompts.md`

**Stage**: 1 (Manual Foundation)
**Duration**: 35 minutes
**Concepts**: 6

### Deliverables

- [x] Lesson markdown file
- [x] Prompt anatomy diagram (Intent → Constraints → Success Criteria)
- [x] Technical action verbs list (Create, Debug, Refactor, Analyze, Optimize, Generate)
- [x] 3 prompt rewrite exercises (vague → specific)

### Content Requirements

**Required Sections**:
1. Prompt structure components explanation
2. Technical action verbs with definitions
3. Specificity principles
4. Socratic questions guiding analysis
5. Manual rewrite exercises (NO "Try With AI")

**Prohibited Content**:
- ❌ NO context window management
- ❌ NO AI tool usage
- ❌ NO Python examples
- ❌ NO Stage/Layer labels in student text

### Validation Criteria

- [ ] 6 concepts present (structure, verbs, specificity, constraints, success criteria, anti-patterns)
- [ ] Socratic questions guide discovery (not lecture-style)
- [ ] Practice vehicle = Bash command prompts (conceptual analysis)
- [ ] Cognitive load = 6 concepts (within A2 limit of 5-7)
- [ ] ZERO AI sections (Stage 1 = manual only)

### Anti-Pattern Checks

- [ ] ✅ No context engineering
- [ ] ✅ No Python code
- [ ] ✅ Socratic modality (not direct teaching like Chapter 9)
- [ ] ✅ No "8-element framework" references

---

## TASK 3: Implement Lesson 3 - Iterative Prompt Refinement

**File**: `book-source/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/03-iterative-prompt-refinement.md`

**Stage**: 2 (AI Collaboration)
**Duration**: 45 minutes
**Concepts**: 7

### Deliverables

- [x] Lesson markdown file
- [x] Three Roles framework demonstration (ALL roles shown)
- [x] Iteration loop example (Git commit messages)
- [x] "Try With AI" section (5-part active collaboration)

### Content Requirements

**Required Sections**:
1. Three Roles framework overview (Teacher/Student/Co-Worker)
2. Iteration loop pattern explanation
3. Practice session: Git commit message refinement
4. **MANDATORY**: "Try With AI" section showing all Three Roles

**Three Roles Demonstration** (NON-NEGOTIABLE):
- ✅ **AI as Teacher**: AI suggests conventional commit format student didn't know
- ✅ **AI as Student**: Student teaches project-specific commit style to AI
- ✅ **AI as Co-Worker**: 3-5 iteration cycles converge on optimal prompt

**Prohibited Content**:
- ❌ NO context window concepts
- ❌ NO Python code
- ❌ NO passive AI tool presentation ("AI will do X for you")
- ❌ NO meta-commentary ("What to notice: AI is teaching you")

### Validation Criteria

- [ ] 7 concepts present (Three Roles, iteration, convergence, bidirectional learning)
- [ ] ALL Three Roles explicitly demonstrated in narrative
- [ ] "Try With AI" section = 5 parts (Request → Evaluate → Teach → Iterate → Reflect)
- [ ] Practice vehicle = Git commit messages (appropriate for Part 3)
- [ ] Cognitive load = 7 concepts (within B1 limit of 7-10)
- [ ] ONLY final section = "Try With AI" (no "Summary", "What's Next")

### Anti-Pattern Checks

- [ ] ✅ Three Roles shown through NARRATIVE (not explicit headers "## AI as Teacher")
- [ ] ✅ No meta-commentary ("What to notice")
- [ ] ✅ No scaffolding labels ("This is Stage 2")
- [ ] ✅ Bidirectional learning visible (not passive AI usage)

---

## TASK 4: Implement Lesson 4 - Specification-First Prompting

**File**: `book-source/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/04-specification-first-prompting.md`

**Stage**: 2 (AI Collaboration)
**Duration**: 40 minutes
**Concepts**: 7

### Deliverables

- [x] Lesson markdown file
- [x] Specification template (Intent → Constraints → Success → Non-Goals)
- [x] Bash script specification example
- [x] "Try With AI" section with specification refinement

### Content Requirements

**Required Sections**:
1. Specification-first principle (WHAT before HOW)
2. Jake Heller's "define what good looks like" framework [15:27-18:16]
3. Specification writing exercise (file backup script)
4. Three Roles in specification refinement
5. "Try With AI" section

**Three Roles Demonstration**:
- ✅ AI teaches: Suggests validation criteria student forgot
- ✅ Student teaches: Specifies project constraints
- ✅ Co-Worker: Spec improves through iteration

### Validation Criteria

- [ ] 7 concepts present (spec-first, success criteria, acceptance tests, constraints, non-goals, format, validation)
- [ ] Jake Heller quote cited [15:27-18:16]
- [ ] Practice vehicle = Bash script spec (no Python)
- [ ] Three Roles demonstrated through narrative
- [ ] "Try With AI" = active collaboration (not passive instructions)

### Anti-Pattern Checks

- [ ] ✅ No context engineering
- [ ] ✅ No Python code
- [ ] ✅ Specification-first modality maintained
- [ ] ✅ No framework label headers

---

## TASK 5: Implement Lesson 5 - Question-Driven Development

**File**: `book-source/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/05-question-driven-development.md`

**Stage**: 2 (AI Collaboration)
**Duration**: 40 minutes
**Concepts**: 7

### Deliverables

- [x] Lesson markdown file
- [x] QDD pattern explanation
- [x] Documentation generation example (AI asks questions)
- [x] "Try With AI" section with QDD practice

### Content Requirements

**Required Sections**:
1. Question-Driven Development concept
2. Meta-prompting (prompting AI to ask questions)
3. Clarifying question types
4. Practice: Documentation generation with QDD
5. "Try With AI" section

**Three Roles Demonstration**:
- ✅ AI teaches: Asks questions student didn't consider
- ✅ Student teaches: Provides project-specific answers
- ✅ Co-Worker: Questions + answers converge on requirements

### Validation Criteria

- [ ] 7 concepts present (QDD, meta-prompting, question types, answers, iteration, elicitation, co-creation)
- [ ] Practice vehicle = documentation for Bash script (appropriate)
- [ ] Three Roles demonstrated
- [ ] "Try With AI" section = active QDD session

### Anti-Pattern Checks

- [ ] ✅ No context window concepts
- [ ] ✅ No Python examples
- [ ] ✅ Socratic modality (questions guide learning)
- [ ] ✅ No passive AI presentation

---

## TASK 6: Implement Lesson 6 - Creating Reusable Prompt Templates

**File**: `book-source/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/06-creating-prompt-templates.md`

**Stage**: 3 (Intelligence Design)
**Duration**: 45 minutes
**Concepts**: 8

### Deliverables

- [x] Lesson markdown file
- [x] 3 prompt template examples (bug fix, refactoring, documentation)
- [x] Template structure guide (placeholders, variables, instructions)
- [x] Template library folder structure
- [x] "Try With AI" section for template testing

### Content Requirements

**Required Sections**:
1. Prompt templates concept (reusable patterns)
2. Template structure with placeholders
3. Template categories for common tasks
4. Three template examples (complete markdown files)
5. "Try With AI" section for validation

**Intelligence Creation** (Stage 3):
- Templates = reusable skills (2-4 decision points each)
- Students create personal template library
- Templates applicable to future development

### Validation Criteria

- [ ] 8 concepts present (templates, structure, placeholders, categories, documentation, testing, refinement)
- [ ] 3 complete template examples provided
- [ ] Templates use Bash/Git/Markdown (NO Python)
- [ ] Template library structure defined
- [ ] "Try With AI" = template testing and refinement

### Anti-Pattern Checks

- [ ] ✅ No context engineering
- [ ] ✅ Templates general enough for reuse (not overly specific)
- [ ] ✅ 2-4 decision points per template (matches skill complexity)
- [ ] ✅ No Python code in templates

---

## TASK 7: Implement Lesson 7 - Template Usage Criteria

**File**: `book-source/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/07-template-usage-criteria.md`

**Stage**: 3 (Intelligence Design)
**Duration**: 40 minutes
**Concepts**: 7

### Deliverables

- [x] Lesson markdown file
- [x] Template Selection Guide (decision tree)
- [x] Usage criteria for each template
- [x] Anti-patterns (when NOT to use templates)
- [x] "Try With AI" section for decision validation

### Content Requirements

**Required Sections**:
1. Decision frameworks (not just lists)
2. Task characterization guide
3. Selection criteria decision tree
4. Template comparison matrix
5. Anti-patterns and edge cases
6. "Try With AI" section

**Intelligence Creation** (Stage 3):
- Decision framework = skill for template selection
- Enables autonomous template usage
- Reusable across all projects

### Validation Criteria

- [ ] 7 concepts present (frameworks, characterization, criteria, comparison, adaptation, anti-patterns, composition)
- [ ] Decision tree covers common scenarios
- [ ] Anti-patterns explicitly defined
- [ ] "Try With AI" = decision validation

### Anti-Pattern Checks

- [ ] ✅ No context engineering
- [ ] ✅ Decision framework is reasoning-based (not rule-based)
- [ ] ✅ No overly specific criteria (general/reusable)

---

## TASK 8: Implement Lesson 8 - Capstone Specification

**File**: `book-source/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/08-capstone-template-library-spec.md`

**Stage**: 4 (Spec-Driven Integration)
**Duration**: 50 minutes
**Concepts**: 0 new (synthesis only)

### Deliverables

- [x] Lesson markdown file
- [x] Specification template (Evals → Intent → Constraints → Non-Goals → Acceptance)
- [x] Example capstone specification
- [x] Peer review checklist
- [x] "Try With AI" section for spec validation

### Content Requirements

**Required Sections**:
1. Specification structure review
2. Capstone project requirements (Template Library Tool)
3. Specification writing exercise
4. Peer review validation
5. "Try With AI" section

**Stage 4 Characteristics**:
- ✅ Orchestrates L6 templates + L7 decision framework
- ✅ Specification ONLY (NO implementation code)
- ✅ Peer-reviewable (validates completeness)

**NO NEW CONCEPTS**: Synthesis of L1-7 only

### Validation Criteria

- [ ] Specification template provided
- [ ] Example capstone spec complete (Evals/Intent/Constraints/Non-Goals/Acceptance)
- [ ] Peer review checklist defined
- [ ] NO implementation code (specification only)
- [ ] "Try With AI" = spec validation with AI feedback

### Anti-Pattern Checks

- [ ] ✅ No context engineering
- [ ] ✅ No implementation code (spec-driven, not implementation)
- [ ] ✅ Orchestrates accumulated intelligence from L6-7
- [ ] ✅ Professional-grade specification demonstrated

---

## TASK 9: Create Chapter README.md

**File**: `book-source/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/README.md`

### Deliverables

- [x] Chapter README with YAML frontmatter
- [x] Chapter overview
- [x] Learning objectives (9 LOs from spec)
- [x] Lesson list with descriptions
- [x] Prerequisites
- [x] Skills taught metadata (institutional integration)
- [x] Research foundation citations

### Content Requirements

**Required Sections**:
1. YAML frontmatter (title, chapter, part, sidebar_position, skills_taught)
2. Chapter overview (core thesis, separation from Ch11)
3. Learning objectives (LO-001 through LO-009)
4. Lesson list (8 lessons with brief descriptions)
5. Prerequisites (Chapters 7-9)
6. Research foundation (Jake Heller citations)
7. Success criteria

**Skills Metadata** (Hidden from students):
```yaml
skills_taught:
  - name: "Prompt Engineering as Specification Skill"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student writes specification-first prompts producing 90%+ satisfactory AI outputs"
  # ... continue for all skills
```

### Validation Criteria

- [ ] All 9 learning objectives listed
- [ ] Jake Heller framework cited with URL
- [ ] Clear separation from Chapter 11 stated
- [ ] Prerequisites accurate (Chapters 7-9)
- [ ] Skills metadata complete (institutional use)

### Anti-Pattern Checks

- [ ] ✅ No unverified statistics
- [ ] ✅ No "8-element framework" references
- [ ] ✅ No context engineering in scope

---

## TASK 10: Validation and Quality Assurance

### Validation Checklist

**Constitutional Compliance**:
- [ ] All lessons follow 4-Stage progression (L1-2: Stage 1, L3-5: Stage 2, L6-7: Stage 3, L8: Stage 4)
- [ ] Stage 1 lessons (L1-2) have ZERO AI collaboration sections
- [ ] Stage 2 lessons (L3-5) demonstrate ALL Three Roles in each lesson
- [ ] Stage 3 lessons (L6-7) create reusable intelligence (templates + criteria)
- [ ] Stage 4 lesson (L8) is specification-only (NO implementation code)
- [ ] All lessons end with ONLY "Try With AI" OR manual exercise (Stage 1)
- [ ] NO lessons have "What's Next", "Summary", "Key Takeaways" sections

**Factual Accuracy**:
- [ ] Jake Heller quotes verified with timestamps [20:03], [15:27-18:16]
- [ ] YouTube URL cited: https://www.youtube.com/watch?v=l0h3nAW13ao
- [ ] NO unverified statistics ("55% productive" removed)
- [ ] NO hallucinated frameworks ("8-element" removed)
- [ ] All tool references accurate (Claude Code, Gemini CLI)

**Separation from Chapter 11**:
- [ ] ZERO context window explanations in any lesson
- [ ] ZERO token counting content
- [ ] ZERO progressive loading strategies
- [ ] ZERO memory file architecture
- [ ] ZERO session management content
- [ ] Focus ONLY on prompt content (not context management)

**Practice Vehicle Compliance**:
- [ ] ALL examples use Bash, Git, Markdown, or conceptual exercises
- [ ] ZERO Python code examples (Part 3 = pre-coding)
- [ ] Examples appropriate for A2-B1 tier

**Cognitive Load Validation**:
- [ ] L1: 5 concepts (✅ A2 limit: 5-7)
- [ ] L2: 6 concepts (✅ A2 limit: 5-7)
- [ ] L3: 7 concepts (✅ B1 limit: 7-10)
- [ ] L4: 7 concepts (✅ B1 limit: 7-10)
- [ ] L5: 7 concepts (✅ B1 limit: 7-10)
- [ ] L6: 8 concepts (✅ B1 limit: 7-10)
- [ ] L7: 7 concepts (✅ B1 limit: 7-10)
- [ ] L8: 0 new concepts (✅ synthesis only)

**Student-Facing Language Protocol**:
- [ ] NO meta-commentary in any lesson ("What to notice: AI is teaching you")
- [ ] NO scaffolding labels ("This is Stage 2", "Layer 1 Focus")
- [ ] NO framework headers ("## Three Roles in Action")
- [ ] Students EXPERIENCE pedagogy through narrative (not study it)

**Anti-Convergence Validation**:
- [ ] Teaching modality differs from Chapter 9 (spec-first + Socratic vs direct teaching)
- [ ] Variety within chapter (4 modalities: spec-first, Socratic, design, spec-driven)
- [ ] NOT all lecture-style (avoid Chapter 9 pattern)

**Three Roles Validation** (L3-5 ONLY):
- [ ] L3: All Three Roles demonstrated ✅
- [ ] L4: All Three Roles demonstrated ✅
- [ ] L5: All Three Roles demonstrated ✅
- [ ] Roles shown through NARRATIVE (not explicit headers)
- [ ] Bidirectional learning visible (AI teaches + learns)
- [ ] Convergence loops present (iteration improves both)

### Quality Metrics

**Completeness**:
- [ ] 8 lessons implemented
- [ ] 1 README implemented
- [ ] All deliverables present (templates, exercises, examples)
- [ ] All learning objectives covered (LO-001 through LO-009)

**Reusability**:
- [ ] Prompt templates applicable to future chapters (Part 4+)
- [ ] Decision framework universal (all AI collaboration)
- [ ] Skills accumulate (templates persist)

**Professional Quality**:
- [ ] Jake Heller framework properly integrated
- [ ] Specification-first approach demonstrated throughout
- [ ] Capstone validates professional competency (peer-reviewable spec)

---

## Implementation Order

**Sequential Execution** (validate each before proceeding):

1. **TASK 1**: Lesson 1 (Foundation, manual only)
2. **TASK 2**: Lesson 2 (Foundation, manual only)
3. **TASK 3**: Lesson 3 (AI Collaboration, Three Roles introduction)
4. **TASK 4**: Lesson 4 (AI Collaboration, specification-first)
5. **TASK 5**: Lesson 5 (AI Collaboration, QDD)
6. **TASK 6**: Lesson 6 (Intelligence Design, templates)
7. **TASK 7**: Lesson 7 (Intelligence Design, criteria)
8. **TASK 8**: Lesson 8 (Spec-Driven, capstone)
9. **TASK 9**: Chapter README
10. **TASK 10**: Validation (comprehensive quality check)

**Validation Gates**:
- After L2: Validate Stage 1 (manual foundation) compliance
- After L5: Validate Stage 2 (Three Roles) compliance
- After L7: Validate Stage 3 (intelligence creation) compliance
- After L8: Validate Stage 4 (spec-driven) compliance
- After README: Validate chapter-level coherence
- Final: Comprehensive constitutional compliance

---

## Success Criteria

**Tasks succeed when**:
- ✅ All 8 lessons implemented following plan
- ✅ All constitutional requirements met
- ✅ All factual claims verified with sources
- ✅ Zero overlap with Chapter 11 (context engineering)
- ✅ Practice vehicles appropriate (Bash/Git/Markdown only)
- ✅ Three Roles demonstrated in L3-5
- ✅ Templates and decision framework reusable
- ✅ Capstone specification peer-reviewable

**Tasks fail when**:
- ❌ Context engineering content appears
- ❌ Python code examples used
- ❌ Unverified claims remain
- ❌ Stage progression violated
- ❌ Meta-commentary exposes scaffolding
- ❌ Teaching modality same as Chapter 9

---

**Tasks Version**: 1.0
**Ready for**: Implementation phase
