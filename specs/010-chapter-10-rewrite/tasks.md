# Chapter 10 Rewrite Tasks: Implementation Checklist

**Specification**: specs/010-chapter-10-rewrite/spec.md v2.0 (Collaboration-Focused)
**Plan**: specs/010-chapter-10-rewrite/plan.md v2.0 (Collaboration-Focused)
**Created**: 2025-11-18
**Revised**: 2025-11-18 (Alignment with approved spec v2.0)

---

## Task Overview

**Total Tasks**: 11 (8 lessons + 1 README + 1 validation + 1 cleanup)

**Implementation Strategy**: Sequential (L1 → L8 → README → Validation)

**Deliverables**:

- 8 lesson markdown files (REPLACE existing 01-08)
- 1 chapter README.md (UPDATE existing)
- Validation report
- Cleanup of old files

---

## TASK 1: Implement Lesson 1 - Why Prompting Matters

**File**: `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/01-why-prompting-matters.md`

**Layer**: 1 (Manual Foundation)
**Duration**: 30 minutes
**Proficiency**: A2
**Concepts**: 3

### Deliverables

- [ ] Lesson markdown file with YAML frontmatter
- [ ] 5 prompt pairs (vague vs. specific comparisons)
- [ ] Manual improvement exercise (NO AI usage)
- [ ] "Try With AI" section (test improved prompts)

### Content Requirements

**Required Sections**:

1. Introduction: Real scenario (asking for "help" vs. asking specific question)
2. Section 1: Compare 5 prompt pairs (vague vs. specific)
3. Section 2: Why clarity matters (AI needs context)
4. Practice: Improve 3 vague prompts WITHOUT AI (manual exercise)
5. "Try With AI": Test improved prompts, compare results

**Core Concepts**:

1. Vague prompts → Poor AI responses (comparison exercise)
2. Specific prompts → Better AI responses (before/after)
3. AI as collaborator, not magic oracle (mental model)

**Practice Vehicle**: Bash command explanations

- Vague: "help with ls"
- Specific: "explain ls -la output showing file permissions and sizes"

**Prohibited Content**:

- ❌ NO specification writing terminology
- ❌ NO Jake Heller framework (comes in L4)
- ❌ NO professional PM language
- ❌ NO context window explanations
- ❌ NO Python code examples
- ❌ NO meta-commentary

### Validation Criteria

- [ ] 3 concepts present (vague→poor, specific→better, AI as collaborator)
- [ ] 5 prompt pairs with clear comparison
- [ ] Manual exercise (NO AI usage in practice section)
- [ ] "Try With AI" is ONLY final section
- [ ] Cognitive load = 3 concepts (within A2 limit of 5-7)
- [ ] Practice vehicle = Bash commands (appropriate for Part 3)

### Anti-Pattern Checks

- [ ] ✅ No specification writing language
- [ ] ✅ No context engineering content
- [ ] ✅ No Python examples
- [ ] ✅ No professional frameworks
- [ ] ✅ Comparison-based modality (not lecture-style)

---

## TASK 2: Implement Lesson 2 - Basic Prompt Structure

**File**: `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/02-basic-prompt-structure.md`

**Layer**: 1 (Manual Foundation)
**Duration**: 35 minutes
**Proficiency**: A2
**Concepts**: 4

### Deliverables

- [ ] Lesson markdown file
- [ ] Task + Context + Format structure explanation
- [ ] Action verbs list (explain, create, debug, optimize)
- [ ] 3 structured prompts (Bash, Git, Markdown tasks)
- [ ] "Try With AI" section

### Content Requirements

**Required Sections**:

1. Opening: Why structure matters (AI understands patterns)
2. Section 1: Task element (action verbs)
3. Section 2: Context element (background information)
4. Section 3: Format element (output structure)
5. Practice: Write 3 prompts using structure (manual, no AI)
6. "Try With AI": Test prompts, evaluate responses

**Core Concepts**:

1. Task (WHAT you want): Verb-first (explain, create, debug, optimize)
2. Context (Background AI needs): Project type, experience level
3. Format (HOW you want answer): Code block, bullet points, table
4. Three-element pattern: Task + Context + Format

**Practice Vehicle**: Git explanations

- Task: "Explain git rebase"
- Context: "assuming I understand merge"
- Format: "in 3 bullet points with examples"

**Prohibited Content**:

- ❌ NO "Intent → Constraints → Success Criteria" (specification language)
- ❌ NO specification writing terminology
- ❌ NO context window management
- ❌ NO Python examples
- ❌ NO meta-commentary

### Validation Criteria

- [ ] 4 concepts present (Task, Context, Format, three-element pattern)
- [ ] Structure shown through examples (not abstract theory)
- [ ] Practice vehicle = Git commands (appropriate for Part 3)
- [ ] Manual exercise BEFORE "Try With AI"
- [ ] Cognitive load = 4 concepts (within A2 limit of 5-7)
- [ ] "Try With AI" is ONLY final section

### Anti-Pattern Checks

- [ ] ✅ No specification writing language (Intent/Constraints/Success)
- [ ] ✅ No context engineering
- [ ] ✅ No Python code
- [ ] ✅ Comparison-based modality (structured vs. unstructured)

---

## TASK 3: Implement Lesson 3 - Adding Examples and Constraints

**File**: `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/03-adding-examples-constraints.md`

**Layer**: 1 (Manual Foundation)
**Duration**: 40 minutes
**Proficiency**: A2
**Concepts**: 5

### Deliverables

- [ ] Lesson markdown file
- [ ] Examples and constraints explanation
- [ ] Enhancement practice (basic → enhanced prompts)
- [ ] "Try With AI" section

### Content Requirements

**Required Sections**:

1. Opening: Why examples and constraints matter
2. Section 1: Providing examples (show don't tell)
3. Section 2: Adding constraints (length, style, level)
4. Section 3: Combining both (examples + constraints)
5. Practice: Enhance L2 prompts with examples/constraints (manual)
6. "Try With AI": Compare basic vs. enhanced prompts

**Core Concepts**:

1. Examples show desired output style
2. Constraints limit scope (length, complexity, language)
3. Multiple examples improve consistency
4. Constraints prevent unwanted behavior
5. Iterative refinement (manual practice)

**Practice Vehicle**: Markdown documentation

- Basic: "Create markdown doc for project README"
- Enhanced: "Create markdown README similar to [example], max 200 words, beginner-friendly language, include installation and usage sections"

**Prohibited Content**:

- ❌ NO specification writing terminology
- ❌ NO context engineering
- ❌ NO Python examples
- ❌ NO meta-commentary

### Validation Criteria

- [ ] 5 concepts present (examples, constraints, consistency, prevention, refinement)
- [ ] Enhancement practice (basic → enhanced)
- [ ] Practice vehicle = Markdown docs (appropriate for Part 3)
- [ ] Manual exercise BEFORE "Try With AI"
- [ ] Cognitive load = 5 concepts (within A2 limit of 5-7)

### Anti-Pattern Checks

- [ ] ✅ No specification language
- [ ] ✅ No context engineering
- [ ] ✅ No Python code
- [ ] ✅ Enhancement practice modality (not lecture)

---

## TASK 4: Implement Lesson 4 - Iterative Refinement

**File**: `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/04-iterative-refinement.md`

**Layer**: 2 (AI Collaboration)
**Duration**: 40 minutes
**Proficiency**: A2→B1 transition
**Concepts**: 5

### Deliverables

- [ ] Lesson markdown file
- [ ] Three Roles demonstration (experienced naturally, NOT labeled)
- [ ] Jake Heller sidebar (60%→97% reference)
- [ ] 3-iteration practice session
- [ ] "Try With AI" section (5-part collaboration)

### Content Requirements

**Required Sections**:

1. Opening: Why iteration matters (Jake Heller 60%→97% sidebar)
2. Section 1: Initial prompt → Evaluate response → Identify gaps
3. Section 2: Refine prompt → Better response → Convergence
4. Section 3: Three Roles in action (experienced, NOT labeled)
5. Practice: 3-iteration session (start → refine → refine → final)
6. "Try With AI": Full iteration loop on Bash script debugging

**Core Concepts**:

1. First prompt rarely perfect (iteration mindset)
2. AI as Teacher (suggests improvements you didn't know)
3. AI as Student (you teach AI your context)
4. AI as Co-Worker (iteration produces better result)
5. Convergence loop (human + AI = better than either alone)

**Three Roles Experienced** (NOT labeled as framework):

- Narrative shows AI suggesting patterns (Teacher role naturally)
- Narrative shows student correcting AI (Student role naturally)
- Narrative shows convergence through iteration (Co-Worker naturally)

**Practice Vehicle**: Bash script debugging

- Iteration 1: "Debug this script" → Generic response
- Iteration 2: "Debug this script; it fails with 'permission denied' on line 5" → Specific fix
- Iteration 3: "Explain WHY this permission issue happens" → Understanding

**Jake Heller Reference** (Sidebar ONLY):

```markdown
> **Expert Insight**: Jake Heller, founder of Casetext (acquired by Thomson Reuters for $650M), spent weeks refining AI prompts from 60% accuracy to 97%+ through iterative refinement. [Source: Y Combinator talk, 2024, timestamp 20:03]
```

**Prohibited Content**:

- ❌ NO framework headers ("## Three Roles Framework")
- ❌ NO meta-commentary ("Notice how AI teaches you")
- ❌ NO scaffolding labels ("This is Layer 2")
- ❌ NO Jake Heller framework as primary teaching tool
- ❌ NO specification writing terminology
- ❌ NO context engineering
- ❌ NO Python examples

### Validation Criteria

- [ ] 5 concepts present (iteration, Teacher, Student, Co-Worker, convergence)
- [ ] Three Roles demonstrated through narrative (NOT labeled)
- [ ] Jake Heller as sidebar (NOT primary framework)
- [ ] 3-iteration practice session included
- [ ] "Try With AI" section with reflection prompts
- [ ] Cognitive load = 5 concepts (within B1 limit of 7-10)
- [ ] Bidirectional learning visible (AI teaches + learns)

### Anti-Pattern Checks

- [ ] ✅ Three Roles experienced (NOT taught as framework)
- [ ] ✅ No explicit headers: "## AI as Teacher"
- [ ] ✅ No meta-commentary exposing scaffolding
- [ ] ✅ Jake Heller as sidebar (not primary)
- [ ] ✅ Experiential modality (not lecture)

---

## TASK 5: Implement Lesson 5 - Question-Driven Development

**File**: `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/05-question-driven-development.md`

**Layer**: 2 (AI Collaboration)
**Duration**: 45 minutes
**Proficiency**: B1
**Concepts**: 6

### Deliverables

- [ ] Lesson markdown file
- [ ] Question-Driven Development explanation
- [ ] Git workflow design example
- [ ] "Try With AI" section (QDD practice)

### Content Requirements

**Required Sections**:

1. Opening: AI as technical consultant (asks before solving)
2. Section 1: Prompting AI to ask questions
3. Section 2: Answering questions thoughtfully
4. Section 3: Compare direct vs. question-driven results
5. Practice: Complex task (git workflow design) using QDD
6. "Try With AI": QDD prompt for git workflows

**Core Concepts**:

1. AI can ask YOU questions before answering
2. Questions reveal assumptions and gaps
3. Your answers provide context AI needs
4. Question-driven produces tailored solutions
5. Comparison: direct prompt vs. question-driven
6. When to use question-driven (complex tasks)

**Practice Vehicle**: Git workflow design

- Direct: "Explain git workflows" → Generic answer
- QDD: "Before explaining git workflows, ask me 5 questions about my experience and project needs" → AI asks → Tailored answer

**Prohibited Content**:

- ❌ NO specification writing terminology
- ❌ NO context engineering
- ❌ NO Python examples
- ❌ NO meta-commentary

### Validation Criteria

- [ ] 6 concepts present (AI asks, reveals gaps, provides context, tailored, comparison, when to use)
- [ ] Comparison: direct vs. QDD approaches
- [ ] Practice vehicle = Git workflows (appropriate)
- [ ] "Try With AI" section with QDD practice
- [ ] Cognitive load = 6 concepts (within B1 limit of 7-10)

### Anti-Pattern Checks

- [ ] ✅ No specification language
- [ ] ✅ No context engineering
- [ ] ✅ No Python code
- [ ] ✅ Experiential + comparison modality

---

## TASK 6: Implement Lesson 6 - Validating AI Outputs

**File**: `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/06-validating-ai-outputs.md`

**Layer**: 2 (AI Collaboration)
**Duration**: 40 minutes
**Proficiency**: B1
**Concepts**: 6

### Deliverables

- [ ] Lesson markdown file
- [ ] Validation checklist (Read → Understand → Test → Question)
- [ ] Bash script validation example
- [ ] "Try With AI" section

### Content Requirements

**Required Sections**:

1. Opening: Why validation matters (AI makes mistakes)
2. Section 1: Validation checklist (systematic approach)
3. Section 2: Red flag patterns (security, complexity, errors)
4. Section 3: Prompting for explanations
5. Practice: Review AI-generated Bash script, identify issues, prompt fixes
6. "Try With AI": Validate and improve AI-generated solution

**Core Concepts**:

1. Never trust AI blindly (validation mindset)
2. Validation checklist: Read → Understand → Test → Question
3. Red flags: Overly complex, missing error handling, unexplained magic
4. Ask AI to explain unfamiliar parts
5. Test outputs in safe environment
6. Iterate when validation fails

**Practice Vehicle**: Bash script validation

- AI generates script with hardcoded path → Student identifies red flag
- Student prompts: "Why is this path hardcoded? How can I make it configurable?"
- AI explains and provides improved version

**Prohibited Content**:

- ❌ NO specification writing terminology
- ❌ NO context engineering
- ❌ NO Python examples
- ❌ NO meta-commentary

### Validation Criteria

- [ ] 6 concepts present (never trust, checklist, red flags, ask, test, iterate)
- [ ] Validation checklist provided
- [ ] Red flag patterns defined
- [ ] Practice vehicle = Bash script (appropriate)
- [ ] "Try With AI" section with validation practice
- [ ] Cognitive load = 6 concepts (within B1 limit of 7-10)

### Anti-Pattern Checks

- [ ] ✅ No specification language
- [ ] ✅ No context engineering
- [ ] ✅ No Python code
- [ ] ✅ Critical analysis modality

---

## TASK 7: Implement Lesson 7 - Building Your Prompt Library

**File**: `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/07-building-prompt-library.md`

**Layer**: 3 (Intelligence Design)
**Duration**: 45 minutes
**Proficiency**: B1
**Concepts**: 7

### Deliverables

- [ ] Lesson markdown file
- [ ] 3 prompt template examples (Git commit, Bash debug, Markdown doc)
- [ ] Template structure guide (placeholders with [VARIABLE] syntax)
- [ ] "Try With AI" section (test templates)

### Content Requirements

**Required Sections**:

1. Opening: Why build library (efficiency + consistency)
2. Section 1: Identify recurring tasks (Bash, Git, Markdown)
3. Section 2: Extract template structure (placeholders)
4. Section 3: Document usage criteria (when to use)
5. Practice: Create 3 templates
6. "Try With AI": Test templates on new scenarios, refine

**Core Concepts**:

1. Recognize recurring tasks (pattern recognition)
2. Extract reusable structure (template creation)
3. Placeholders for variables (generalization)
4. Usage notes for context (decision guide)
5. Template categories (explain, debug, create, optimize)
6. Version control for templates (iterate and improve)
7. Test templates on new scenarios (validation)

**Practice Vehicle**: Template creation

- **Git commit template**: "Write commit message for [FEATURE] that [ACTION], following [STYLE]"
- **Bash debug template**: "Debug Bash script [FILENAME] failing with [ERROR] at [LINE]"
- **Markdown doc template**: "Create markdown doc for [PROJECT] including [SECTIONS]"

**Prohibited Content**:

- ❌ NO specification writing terminology
- ❌ NO context engineering
- ❌ NO Python templates (students haven't learned Python yet)
- ❌ NO meta-commentary

### Validation Criteria

- [ ] 7 concepts present (recognize, extract, placeholders, usage, categories, version, test)
- [ ] 3 complete template examples provided
- [ ] Templates use Bash/Git/Markdown (NO Python)
- [ ] Placeholder syntax: [VARIABLE]
- [ ] Usage notes for each template
- [ ] "Try With AI" section for testing
- [ ] Cognitive load = 7 concepts (within B1 limit of 7-10)

### Anti-Pattern Checks

- [ ] ✅ No specification language
- [ ] ✅ No context engineering
- [ ] ✅ No Python templates
- [ ] ✅ Templates reusable (2-4 decision points)
- [ ] ✅ Design-focused modality

---

## TASK 8: Implement Lesson 8 - Capstone: Your First Prompt Toolkit

**File**: `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/08-capstone-prompt-toolkit.md`

**Layer**: 3 (Intelligence Design) + Capstone Integration
**Duration**: 60 minutes
**Proficiency**: B1
**Concepts**: Integration (no new concepts)

### Deliverables

- [ ] Lesson markdown file
- [ ] Toolkit structure template
- [ ] Decision guide framework
- [ ] Assessment criteria
- [ ] "Try With AI" section (validate toolkit)

### Content Requirements

**Required Sections**:

1. Opening: Build personal prompt toolkit (portfolio piece)
2. Section 1: Audit recurring tasks (what do YOU do often?)
3. Section 2: Create 5-7 templates covering tasks
4. Section 3: Decision guide (task → template mapping)
5. Section 4: Test each template on real scenario
6. Practice: Build complete toolkit (markdown file)
7. "Try With AI": Validate toolkit with AI feedback, peer review

**Toolkit Structure**:

```markdown
# My Prompt Toolkit

## Template 1: [Name]

**When to use**: [Criteria]
**Template**: [Prompt with [PLACEHOLDERS]]
**Example**: [Filled-in version]

## Template 2: [Name]

...

## Decision Guide

- For [task type] → Use [template name]
- For [task type] → Use [template name]
```

**Assessment Criteria**:

- Toolkit has 5-7 templates
- Each has usage criteria
- Decision guide included
- Peer can use without clarification

**Prohibited Content**:

- ❌ NO specification writing as deliverable (toolkit, not spec)
- ❌ NO context engineering
- ❌ NO Python templates
- ❌ NO meta-commentary

### Validation Criteria

- [ ] Toolkit structure template provided
- [ ] Decision guide framework included
- [ ] Assessment criteria defined (5-7 templates, usage criteria, peer-usable)
- [ ] "Try With AI" section for validation
- [ ] NO new concepts (integration only)
- [ ] Deliverable = toolkit (NOT specification document)

### Anti-Pattern Checks

- [ ] ✅ No specification writing as primary deliverable
- [ ] ✅ Focus on toolkit creation (not spec writing)
- [ ] ✅ No context engineering
- [ ] ✅ No Python templates
- [ ] ✅ Synthesis + design modality

---

## TASK 9: Update Chapter README.md

**File**: `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/README.md`

### Deliverables

- [ ] Updated README with YAML frontmatter
- [ ] Chapter overview (collaboration-focused thesis)
- [ ] Learning objectives (LO-001 through LO-008)
- [ ] 8-lesson list with descriptions
- [ ] Prerequisites (Chapters 7-9)
- [ ] Separation from Chapter 11 clearly stated

### Content Requirements

**Required Sections**:

1. YAML frontmatter (update title, description)
2. Chapter overview:
   - Core thesis: "Clear prompts get better AI responses"
   - Collaboration-focused (NOT specification writing)
   - Separation from Chapter 11 (prompt content vs. context management)
3. Learning objectives (8 LOs from spec v2.0)
4. Lesson list (8 lessons with brief descriptions)
5. Prerequisites (Chapters 7-9: Bash, Git, Markdown)

**Core Thesis** (UPDATED):
"Clear prompts get better AI responses. This chapter teaches you how to communicate your intent effectively so AI can help you accomplish tasks. You'll learn to structure prompts clearly, iterate through collaboration, validate AI outputs, and build reusable prompt templates for common development tasks."

**Separation from Chapter 11**:
"This chapter focuses exclusively on **what you SAY** to your AI agent (prompt engineering). Chapter 11 will teach **what your AI agent KNOWS** when you say it (context engineering)."

**Prohibited Content**:

- ❌ NO "specification skill" language
- ❌ NO Jake Heller as primary framework
- ❌ NO professional PM terminology
- ❌ NO unverified statistics
- ❌ NO "8-element framework" references

### Validation Criteria

- [ ] Core thesis = collaboration-focused (NOT specification writing)
- [ ] All 8 learning objectives listed (LO-001 through LO-008)
- [ ] Separation from Chapter 11 clearly stated
- [ ] Prerequisites accurate (Chapters 7-9)
- [ ] 8-lesson structure reflected
- [ ] NO specification writing language in overview

### Anti-Pattern Checks

- [ ] ✅ No "specification skill" terminology
- [ ] ✅ No professional frameworks as primary
- [ ] ✅ No unverified statistics
- [ ] ✅ No context engineering in scope
- [ ] ✅ Collaboration-focused language throughout

---

## TASK 10: Remove Old Lesson Files

**Location**: `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/`

### Deliverables

- [ ] Backup old files (01-08 specification-focused versions)
- [ ] Remove old files from chapter directory
- [ ] Confirm new files (01-08 collaboration-focused) are in place

### Actions Required

1. Backup old files:

   - Move to `specs/010-chapter-10-rewrite/backup-old-lessons/`
   - Preserve for reference

2. Remove from chapter directory:

   - `01-prompts-as-specifications.md` (OLD)
   - `02-anatomy-effective-prompts.md` (OLD)
   - `03-iterative-prompt-refinement.md` (OLD)
   - `04-specification-first-prompting.md` (OLD)
   - `05-question-driven-development.md` (may need update)
   - `06-reusable-prompt-templates.md` (may need update)
   - `07-template-selection-criteria.md` (may need update)
   - `08-capstone-template-library.md` (may need update)

3. Verify new files in place:
   - `01-why-prompting-matters.md` (NEW)
   - `02-basic-prompt-structure.md` (NEW)
   - `03-adding-examples-constraints.md` (NEW)
   - `04-iterative-refinement.md` (UPDATED)
   - `05-question-driven-development.md` (CHECK/UPDATE)
   - `06-validating-ai-outputs.md` (NEW)
   - `07-building-prompt-library.md` (UPDATED)
   - `08-capstone-prompt-toolkit.md` (UPDATED)

### Validation Criteria

- [ ] Old files backed up (not lost)
- [ ] New files in chapter directory
- [ ] 8 lessons total (01-08)
- [ ] No duplicate filenames

---

## TASK 11: Validation and Quality Assurance

### Validation Checklist

**Constitutional Compliance**:

- [ ] All lessons follow 4-Layer progression:
  - L1-3: Layer 1 (Manual Foundation)
  - L4-6: Layer 2 (AI Collaboration)
  - L7-8: Layer 3 (Intelligence Design)
- [ ] Layer 1 lessons (L1-3) have manual practice BEFORE "Try With AI"
- [ ] Layer 2 lessons (L4-6) demonstrate Three Roles naturally (NOT labeled)
- [ ] Layer 3 lessons (L7-8) create reusable intelligence (templates + decision guide)
- [ ] All lessons end with ONLY "Try With AI" section
- [ ] NO lessons have "What's Next", "Summary", "Key Takeaways" sections

**Collaboration-Focused Approach**:

- [ ] Core thesis = "Clear prompts get better AI responses" (NOT specification writing)
- [ ] Task + Context + Format structure (NOT Intent/Constraints/Success)
- [ ] Jake Heller as sidebar in L4 ONLY (NOT primary framework)
- [ ] NO professional PM terminology
- [ ] NO specification writing as primary pedagogy

**Factual Accuracy**:

- [ ] Jake Heller reference as sidebar in L4 (timestamp [20:03])
- [ ] YouTube URL cited: https://www.youtube.com/watch?v=l0h3nAW13ao
- [ ] Anthropic, Google, OpenAI docs cited for prompting guidance
- [ ] NO unverified statistics ("55% productive" removed)
- [ ] NO hallucinated frameworks ("8-element AIDD" removed)

**Separation from Chapter 11**:

- [ ] ZERO context window explanations in any lesson
- [ ] ZERO token counting content
- [ ] ZERO progressive loading strategies
- [ ] ZERO memory file architecture (CLAUDE.md, architecture.md)
- [ ] ZERO session management content
- [ ] Focus ONLY on prompt content (Task + Context + Format)

**Practice Vehicle Compliance**:

- [ ] ALL examples use Bash, Git, Markdown ONLY
- [ ] ZERO Python code examples (Part 3 = pre-programming)
- [ ] Examples appropriate for A2-B1 tier
- [ ] Bash: command explanations, script debugging
- [ ] Git: commit messages, workflow design
- [ ] Markdown: documentation creation

**Cognitive Load Validation**:

- [ ] L1: 3 concepts (✅ A2 limit: 5-7)
- [ ] L2: 4 concepts (✅ A2 limit: 5-7)
- [ ] L3: 5 concepts (✅ A2 limit: 5-7)
- [ ] L4: 5 concepts (✅ A2→B1: 7-10)
- [ ] L5: 6 concepts (✅ B1 limit: 7-10)
- [ ] L6: 6 concepts (✅ B1 limit: 7-10)
- [ ] L7: 7 concepts (✅ B1 limit: 7-10)
- [ ] L8: 0 new concepts (✅ integration only)

**Student-Facing Language Protocol**:

- [ ] NO meta-commentary in any lesson ("Notice how AI teaches you")
- [ ] NO scaffolding labels ("This is Layer 2", "Stage 1 Focus")
- [ ] NO framework headers ("## Three Roles Framework")
- [ ] Students EXPERIENCE collaboration through narrative (not study frameworks)
- [ ] Three Roles in L4-6 experienced naturally (AI suggests, student corrects, convergence happens)

**Anti-Convergence Validation**:

- [ ] Teaching modality differs from Chapter 9:
  - Chapter 9: Direct teaching (lecture-style markdown tutorial)
  - Chapter 10: Experiential collaboration (hands-on practice with reflection)
- [ ] Variety within chapter:
  - L1-3: Comparison-based (vague vs. clear, analyze manually)
  - L4-6: Experiential (hands-on with AI, reflection)
  - L7-8: Design-focused (create templates, build toolkit)
- [ ] NOT lecture-style (avoid Chapter 9 pattern)

**Three Roles Validation** (L4-6 ONLY):

- [ ] L4: Three Roles demonstrated naturally (iteration reveals Teacher/Student/Co-Worker)
- [ ] L5: Three Roles demonstrated naturally (QDD shows roles through questions)
- [ ] L6: Three Roles demonstrated naturally (validation shows roles through iteration)
- [ ] Roles shown through NARRATIVE (not explicit headers)
- [ ] Bidirectional learning visible (AI teaches + learns from student)
- [ ] Convergence loops present (iteration improves understanding)

### Quality Metrics

**Completeness**:

- [ ] 8 lessons implemented (collaboration-focused)
- [ ] 1 README updated (collaboration-focused thesis)
- [ ] All deliverables present (templates, exercises, examples)
- [ ] All learning objectives covered (LO-001 through LO-008)

**Reusability**:

- [ ] Prompt templates applicable to future chapters (Part 4+ Python work)
- [ ] Decision guide universal (all AI collaboration contexts)
- [ ] Templates use Bash/Git/Markdown (expandable to Python later)

**Pedagogical Quality**:

- [ ] Collaboration-focused (NOT specification writing)
- [ ] Experiential modality (hands-on practice, reflection)
- [ ] Progressive: L1 (compare prompts) → L8 (build toolkit)
- [ ] Three Roles experienced naturally (not taught as framework)

---

## Implementation Order

**Sequential Execution** (validate each before proceeding):

1. **TASK 1**: Lesson 1 - Why Prompting Matters (Layer 1, comparison-based)
2. **TASK 2**: Lesson 2 - Basic Prompt Structure (Layer 1, structure analysis)
3. **TASK 3**: Lesson 3 - Adding Examples and Constraints (Layer 1, enhancement)
4. **TASK 4**: Lesson 4 - Iterative Refinement (Layer 2, Three Roles introduction)
5. **TASK 5**: Lesson 5 - Question-Driven Development (Layer 2, QDD)
6. **TASK 6**: Lesson 6 - Validating AI Outputs (Layer 2, critical analysis)
7. **TASK 7**: Lesson 7 - Building Your Prompt Library (Layer 3, templates)
8. **TASK 8**: Lesson 8 - Capstone Toolkit (Layer 3, integration)
9. **TASK 9**: Update Chapter README (collaboration-focused)
10. **TASK 10**: Remove old lesson files (cleanup)
11. **TASK 11**: Validation (comprehensive quality check)

**Validation Gates**:

- After L3: Validate Layer 1 (manual foundation) compliance
- After L6: Validate Layer 2 (collaboration, Three Roles) compliance
- After L8: Validate Layer 3 (intelligence creation) compliance
- After README: Validate chapter-level coherence
- Final: Comprehensive constitutional compliance

---

## Success Criteria

**Tasks succeed when**:

- ✅ All 8 lessons implemented (collaboration-focused approach)
- ✅ Core thesis = "Clear prompts get better AI responses"
- ✅ Task + Context + Format structure (NOT specification writing)
- ✅ Jake Heller as sidebar ONLY (NOT primary framework)
- ✅ Three Roles experienced naturally in L4-6 (NOT labeled)
- ✅ Zero overlap with Chapter 11 (context engineering)
- ✅ Practice vehicles = Bash/Git/Markdown ONLY (NO Python)
- ✅ All factual claims verified with sources
- ✅ Templates and decision guide reusable (Layer 3 intelligence)

**Tasks fail when**:

- ❌ Specification writing as primary pedagogy
- ❌ Professional PM frameworks as primary teaching
- ❌ Jake Heller framework as primary (should be sidebar only)
- ❌ Context engineering content appears
- ❌ Python code examples used
- ❌ Unverified claims remain
- ❌ Meta-commentary exposes scaffolding
- ❌ Three Roles taught as framework (should be experienced)
- ❌ Teaching modality same as Chapter 9

---

**Tasks Version**: 2.0 (Collaboration-Focused, aligned with spec v2.0 and plan v2.0)
**Ready for**: Implementation phase
