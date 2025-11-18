# Chapter 10 Rewrite Plan: Prompt Engineering for AI-Driven Development

**Specification**: specs/010-chapter-10-rewrite/spec.md (APPROVED)
**Created**: 2025-11-18
**Constitutional Version**: 6.0.1
**Lesson Count**: 8 lessons (derived from concept density analysis)

---

## I. Pedagogical Architecture

### Concept Density Analysis

**Total Concepts**: 12 distinct prompt engineering concepts

**Chunking Analysis**:
1. **Foundation** (4 concepts): Prompts as specs, structure, verbs, clarity → 2 lessons
2. **Application** (5 concepts): Iteration, Three Roles, convergence, QDD, refinement → 3 lessons
3. **Integration** (2 concepts): Templates, usage criteria → 2 lessons
4. **Mastery** (1 concept): Template orchestration → 1 lesson

**Lesson Count Justification**: 8 lessons
- NOT arbitrary (not forcing into 9-lesson pattern)
- Derived from B1 cognitive load (7-10 concepts per lesson max)
- Respects pedagogical phases (Foundation → Application → Integration → Mastery)
- Matches 4-Stage framework (2 + 3 + 2 + 1 = 8)

### Pedagogical Progression

**Foundation Phase** (Lessons 1-2):
- **Purpose**: Build mental models manually (Stage 1: NO AI)
- **Cognitive load**: A2 tier (5-7 concepts per lesson)
- **Teaching modality**: Specification-first (show WHAT prompts achieve BEFORE HOW)
- **Outcome**: Students can analyze prompt quality WITHOUT AI assistance

**Application Phase** (Lessons 3-5):
- **Purpose**: AI collaboration demonstrating Three Roles (Stage 2)
- **Cognitive load**: A2→B1 transition (7 concepts per lesson)
- **Teaching modality**: Socratic dialogue + hands-on iteration
- **Outcome**: Students iterate prompts with AI showing Teacher/Student/Co-Worker dynamics

**Integration Phase** (Lessons 6-7):
- **Purpose**: Create reusable intelligence (Stage 3)
- **Cognitive load**: B1 tier (7-10 concepts per lesson)
- **Teaching modality**: Design-focused (create prompt templates)
- **Outcome**: Students build personal template library with usage criteria

**Mastery Phase** (Lesson 8):
- **Purpose**: Orchestrate via specification (Stage 4)
- **Cognitive load**: B1 tier (synthesis, no new concepts)
- **Teaching modality**: Specification-only capstone
- **Outcome**: Professional-grade template library spec for peer use

### Teaching Modality Strategy (Anti-Convergence)

**Chapter 9 Modality**: Direct teaching (markdown syntax tutorial)

**Chapter 10 Modality** (MUST vary):
- **Specification-first**: Show WHAT prompts achieve BEFORE teaching syntax
- **Socratic dialogue**: Questions guide students to discover patterns
- **NOT**: Lecture-style explanations (avoid Chapter 9 pattern)

**Per-Lesson Modality Assignment**:
- **L1-2**: Specification-first (show excellent prompt results → analyze structure)
- **L3-5**: Socratic + hands-on (questions guide iteration practice)
- **L6-7**: Design-focused (students create templates)
- **L8**: Specification-driven (write spec, NO implementation)

---

## II. Lesson-by-Lesson Breakdown

### Lesson 1: Prompts as Specifications

**Stage**: 1 (Manual Foundation)
**Duration**: 30 minutes
**Proficiency**: A2
**Concepts**: 5

**Learning Objectives**:
- LO-001: Explain prompt engineering as "specification skill" using Jake Heller framework
- Understand: Prompts define WHAT AI should produce (specifications, not commands)

**Concept List**:
1. Prompt engineering as specification writing (Jake Heller context)
2. WHAT vs HOW thinking (intent before implementation)
3. Specification-first principle (define success before prompting)
4. 60% → 97% iteration framework (Heller's CoCounsel experience)
5. Prompt quality = output quality (garbage in, garbage out)

**Teaching Modality**: Specification-first
- Show: Excellent prompt example → resulting code output → analyze structure
- Show: Poor prompt example → problematic output → identify gaps
- Students: Analyze prompt quality WITHOUT running AI

**Practice Vehicle**: Markdown analysis
- Students read 5 prompt examples (markdown documents)
- Manually identify which prompts are specifications vs vague requests
- NO AI tools used (pure manual analysis)

**Prerequisites from Earlier Chapters**:
- Chapter 9: Markdown formatting basics
- Chapter 4: Nine Pillars of AIDD (context for why prompts matter)

**Deliverable**: Handwritten comparison chart (Specification Prompts vs Vague Prompts)

**No "Try With AI" section** (Stage 1: manual only)

---

### Lesson 2: Anatomy of Effective Prompts

**Stage**: 1 (Manual Foundation)
**Duration**: 35 minutes
**Proficiency**: A2
**Concepts**: 6

**Learning Objectives**:
- LO-002: Analyze prompt pairs (vague vs specific) and identify WHY specific prompts succeed
- LO-003: Manually structure prompts using "Intent → Constraints → Success Criteria" pattern

**Concept List**:
1. Prompt structure components (Intent, Constraints, Success Criteria)
2. Technical action verbs (Create, Debug, Refactor, Analyze, Optimize, Generate)
3. Specificity principles (concrete vs abstract language)
4. Constraint types (technical, scope, quality, format)
5. Success criteria definition ("what good looks like" - Heller framework)
6. Anti-pattern recognition (vagueness, ambiguity, missing context)

**Teaching Modality**: Specification-first + Socratic
- Present: Prompt anatomy diagram (Intent → Constraints → Success → Output)
- Socratic questions:
  - "What's missing from this prompt that causes poor output?"
  - "Which constraint would prevent this failure mode?"
  - "How would you define 'success' for this task?"

**Practice Vehicle**: Bash command prompts (conceptual)
- Students see prompts for Bash tasks (file operations, text processing)
- Manually decompose prompts into Intent/Constraints/Success
- Write improved versions on paper/markdown (NO AI execution)

**Prerequisites**:
- Lesson 1: Prompts as specifications
- Chapter 7: Bash basics (understand what commands do)

**Deliverable**: 3 prompt rewrites (vague → specific) with structure annotations

**No "Try With AI" section** (Stage 1: manual only)

---

### Lesson 3: Iterative Prompt Refinement (Three Roles Introduction)

**Stage**: 2 (AI Collaboration)
**Duration**: 45 minutes
**Proficiency**: A2→B1 transition
**Concepts**: 7

**Learning Objectives**:
- LO-004: Demonstrate THREE ROLES framework (AI as Teacher/Student/Co-Worker)
- Apply: Iteration loop (initial prompt → AI feedback → refinement → convergence)

**Concept List**:
1. Three Roles framework overview (Teacher/Student/Co-Worker)
2. AI as Teacher (suggests improvements you didn't know)
3. AI as Student (learns your project constraints)
4. AI as Co-Worker (iterative convergence)
5. Iteration loop pattern (prompt → output → analyze → refine → repeat)
6. Convergence criteria (when to stop iterating)
7. Bidirectional learning (both human and AI improve understanding)

**Teaching Modality**: Socratic + hands-on iteration

**Practice Vehicle**: Git commit message generation
- **Part 1: Initial Prompt** (student writes first attempt)
- **Part 2: AI as Teacher** (AI suggests better structure/conventions)
- **Part 3: AI as Student** (student teaches project-specific commit style)
- **Part 4: Convergence** (iterate until commit message meets standards)

**Prerequisites**:
- Lessons 1-2: Prompt structure understanding
- Chapter 8: Git basics (understand commits)

**Deliverable**:
- Iteration log (3-5 refinement cycles documented)
- Final prompt that produces excellent commit messages

**Three Roles Demonstration** (MANDATORY):
- ✅ AI teaches: Student learns conventional commit format from AI
- ✅ Student teaches: Student corrects AI to match team's style
- ✅ Co-Worker: Multiple iterations converge on optimal prompt

**Try With AI Section**:
```markdown
## Try With AI: Commit Message Refinement

**Part 1: Initial Request**
Ask Claude Code or Gemini CLI: "Write a commit message for my changes"

**Part 2: Critical Evaluation**
Review AI's response. Ask yourself:
- Does this follow conventional commit format?
- Does it capture WHY changes were made (not just WHAT)?
- Is it specific enough for teammate review?

**Part 3: Teach AI Your Standards**
Tell AI: "Use conventional commits format. Include scope. Explain business value, not just technical changes."

**Part 4: Iterate**
Refine prompt until output matches your quality bar.

**Part 5: Reflection**
- What did AI teach you about commit messages?
- What did you teach AI about your project?
- How did the final version improve from iteration?
```

---

### Lesson 4: Specification-First Prompting

**Stage**: 2 (AI Collaboration)
**Duration**: 40 minutes
**Proficiency**: B1
**Concepts**: 7

**Learning Objectives**:
- LO-005: Write specification-first prompts (define WHAT before asking HOW)
- Apply: Jake Heller's "define what good looks like" before prompting

**Concept List**:
1. Specification-first principle (WHAT before HOW)
2. Success criteria definition (measurable outcomes)
3. Acceptance test thinking (how to validate output)
4. Constraints before implementation (boundaries upfront)
5. Non-goals clarification (what we're NOT building)
6. Output format specification (structure requirements)
7. Validation criteria (pass/fail thresholds)

**Teaching Modality**: Specification-first + Socratic

**Practice Vehicle**: Bash script specification
- Students specify "file backup script" requirements BEFORE prompting
- Define: What files, where to backup, error handling, success indicators
- Write specification → THEN prompt AI → validate output against spec

**Prerequisites**:
- Lesson 3: Three Roles framework
- Chapter 7: Bash scripting basics

**Deliverable**:
- Specification document (markdown)
- Prompt based on specification
- Validation report (does output meet spec?)

**Three Roles Demonstration**:
- ✅ AI teaches: Suggests validation criteria student forgot
- ✅ Student teaches: Specifies project-specific constraints
- ✅ Co-Worker: Spec + prompt converge through iteration

**Try With AI Section**: Active collaboration with specification refinement

---

### Lesson 5: Question-Driven Development

**Stage**: 2 (AI Collaboration)
**Duration**: 40 minutes
**Proficiency**: B1
**Concepts**: 7

**Learning Objectives**:
- LO-006: Apply Question-Driven Development (prompt AI to ask clarifying questions)
- Understand: Better inputs from AI questioning produce better outputs

**Concept List**:
1. Question-Driven Development (QDD) pattern
2. Prompting AI to ask questions (meta-prompting)
3. Clarifying question types (scope, constraints, preferences, edge cases)
4. Answer quality (specific vs vague responses)
5. Iterative questioning (follow-up questions)
6. Requirement elicitation (AI helps discover what you need)
7. Specification co-creation (AI + human converge on requirements)

**Teaching Modality**: Socratic + hands-on

**Practice Vehicle**: Documentation generation task
- Instead of: "Write documentation for this Bash script"
- Student prompts: "Ask me 10 clarifying questions before writing documentation"
- AI asks: Audience level? Format? Sections needed? Examples required?
- Student answers → AI generates tailored documentation

**Prerequisites**:
- Lesson 4: Specification-first thinking
- Lessons 3: Three Roles framework

**Deliverable**:
- QDD session log (questions asked, answers given)
- Final documentation output
- Reflection: How questions improved output quality

**Three Roles Demonstration**:
- ✅ AI teaches: Asks questions student didn't think of
- ✅ Student teaches: Provides project-specific answers
- ✅ Co-Worker: Questions + answers converge on requirements

**Try With AI Section**: QDD practice session with reflection prompts

---

### Lesson 6: Creating Reusable Prompt Templates

**Stage**: 3 (Intelligence Design)
**Duration**: 45 minutes
**Proficiency**: B1
**Concepts**: 8

**Learning Objectives**:
- LO-007: Create 3+ reusable prompt templates for common tasks
- Design: Templates as 2-4 decision point skills

**Concept List**:
1. Prompt templates concept (reusable patterns)
2. Template structure (placeholders, variables, instructions)
3. Decision points (what varies per use)
4. Template categories (new feature, bug fix, refactoring, testing, documentation)
5. Placeholder syntax (clear variable marking)
6. Template documentation (when to use, how to customize)
7. Template testing (validate with multiple scenarios)
8. Template refinement (improve through use)

**Teaching Modality**: Design-focused

**Practice Vehicle**: Common development tasks (Bash/Git/Markdown)
- Template 1: Bug fix prompt template
- Template 2: Refactoring prompt template
- Template 3: Documentation prompt template

**Template Structure Example**:
```markdown
# Bug Fix Prompt Template

**INTENT**: Fix bug causing [SYMPTOM]

**CONTEXT**:
- File: [FILEPATH]
- Error: [ERROR_MESSAGE]
- Expected behavior: [EXPECTED]
- Actual behavior: [ACTUAL]

**CONSTRAINTS**:
- MUST preserve existing functionality
- MUST include error handling
- MUST add test case preventing regression

**SUCCESS CRITERIA**:
- Bug no longer reproduces
- Tests pass
- No new bugs introduced
```

**Prerequisites**:
- Lessons 1-5: All prompt engineering foundations
- Understanding of development workflows

**Deliverable**:
- 3 prompt templates (markdown files)
- Usage examples for each template
- Personal template library (folder structure)

**Intelligence Creation** (Stage 3):
- Templates = reusable skills (2-4 decision points each)
- Students customize for their projects
- Templates accumulate across future development

**Try With AI Section**: Test templates with AI, refine based on outputs

---

### Lesson 7: Template Usage Criteria and Decision Frameworks

**Stage**: 3 (Intelligence Design)
**Duration**: 40 minutes
**Proficiency**: B1
**Concepts**: 7

**Learning Objectives**:
- LO-008: Document WHEN to use each template (decision framework, not just library)
- Create: Usage criteria for template selection

**Concept List**:
1. Decision frameworks (not just template lists)
2. Task characterization (identify template fit)
3. Selection criteria (matching task to template)
4. Template comparison (when multiple templates could work)
5. Template adaptation (customizing for edge cases)
6. Anti-patterns (when NOT to use templates)
7. Template composition (combining multiple templates)

**Teaching Modality**: Design-focused + Socratic

**Practice Vehicle**: Create "Template Selection Guide"
- Decision tree: Task characteristics → Template recommendation
- Examples: "New feature" → which template? Why?
- Edge cases: Task doesn't fit existing templates → what to do?

**Decision Framework Structure**:
```markdown
# Template Selection Guide

## Decision Tree

**Question 1**: Is this creating something new or fixing existing?
- New → Go to Q2
- Fixing → Bug Fix Template

**Question 2**: Does this require architectural design?
- Yes → New Feature Template (complex)
- No → Simple Implementation Template

**Question 3**: Is this refactoring or optimizing?
- Refactoring → Refactoring Template
- Optimizing → Performance Template

[Continue decision tree...]

## Anti-Patterns (When NOT to use templates)

- Task is trivial (1-line change) → Direct prompt
- Task is completely novel → Custom prompt from scratch
- Template assumptions don't match context → Adapt or create new
```

**Prerequisites**:
- Lesson 6: Prompt templates created
- All previous lessons (full understanding)

**Deliverable**:
- Template Selection Guide (markdown)
- Usage examples with reasoning
- Template library documentation

**Intelligence Creation** (Stage 3):
- Decision framework = skill for template selection
- Reusable across all future projects
- Enables autonomous template usage

**Try With AI Section**: Validate decision tree with AI, test edge cases

---

### Lesson 8: Capstone - Prompt Template Library Specification

**Stage**: 4 (Spec-Driven Integration)
**Duration**: 50 minutes
**Proficiency**: B1
**Concepts**: 0 new (synthesis only)

**Learning Objectives**:
- LO-009: Write complete template library specification for peer use
- Demonstrate: Professional-grade specification capability

**Concept List** (NO new concepts - synthesis):
- Orchestrates: Templates from L6, Usage criteria from L7
- Applies: Specification-first thinking (L4), QDD (L5), Three Roles (L3)
- Validates: Specification completeness (L1-2 foundations)

**Teaching Modality**: Specification-driven (NO implementation code)

**Practice Vehicle**: Specification document ONLY
- Write spec for "Personal Prompt Template Library Tool"
- Include: Intent, Constraints, Success Criteria, Non-Goals, Acceptance Tests
- Peer review: Another student uses spec to understand system

**Specification Structure**:
```markdown
# Prompt Template Library Specification

## I. Evals (What Success Looks Like)
- Developer can find appropriate template in <30 seconds
- Templates produce 90%+ satisfactory AI outputs
- Decision framework guides selection correctly

## II. Intent (What We're Building)
- Library of 5-7 prompt templates
- Selection decision tree
- Usage documentation
- Example applications

## III. Constraints
- Templates use placeholder syntax: [VARIABLE]
- Templates documented with 3+ usage examples each
- Decision tree covers 95% of common dev tasks
- Markdown-based (portable, version-controllable)

## IV. Non-Goals
- NOT a code generator (templates guide prompting, not replace it)
- NOT template execution automation (students still prompt AI manually)
- NOT domain-specific (general development templates)

## V. Acceptance Tests
- Peer can select correct template for 5 test scenarios
- Peer can customize template without clarification questions
- Templates produce working AI outputs in testing
```

**Prerequisites**:
- ALL Lessons 1-7 (complete foundation)
- Templates from L6, Decision framework from L7

**Deliverable**:
- Complete specification (3-5 pages markdown)
- Peer review validation report
- NO implementation code (specification only)

**Stage 4 Demonstration**:
- ✅ Orchestrates accumulated intelligence (templates + criteria)
- ✅ Specification-only (no implementation)
- ✅ Peer-reviewable (validates specification completeness)

**Try With AI Section**:
```markdown
## Try With AI: Specification Validation

**Part 1: Completeness Check**
Ask AI: "Review this specification. What's missing? What's ambiguous?"

**Part 2: Peer Simulation**
Ask AI: "Pretend you're a developer using this spec. What questions would you have?"

**Part 3: Refinement**
Based on AI feedback, improve specification clarity.

**Part 4: Final Validation**
Confirm: Could a peer implement this without asking you questions?
```

---

## III. Intelligence Accumulation Map

### Skills Created (Stage 3)

**Lesson 6 Deliverables**:
- `bug-fix-template.md` (prompt template with 2-4 decision points)
- `refactoring-template.md` (prompt template)
- `documentation-template.md` (prompt template)
- Template library structure (folder organization)

**Lesson 7 Deliverables**:
- `template-selection-guide.md` (decision framework)
- Usage criteria documentation
- Anti-pattern recognition guide

**Capstone Orchestration (Lesson 8)**:
- Specification orchestrates ALL templates + decision framework
- Peer-reviewable artifact (validates professional competency)

### Reusability Across Book

**Templates Apply To**:
- Part 4 (Python Fundamentals): Bug fixes, refactoring, testing
- Part 5 (Spec-Driven Development): Specification writing
- Part 6+ (Advanced topics): Domain-specific template adaptation

**Decision Framework Applies To**:
- All future AI collaboration (universal skill)
- Professional development workflows
- Team collaboration (shareable templates)

---

## IV. Anti-Convergence Validation

### Teaching Modality Comparison

**Chapter 9** (Markdown):
- Modality: Direct teaching (lecture-style syntax tutorial)
- Approach: "Here's how markdown works" → demonstrations → practice
- Student role: Passive learner following instructions

**Chapter 10** (Prompt Engineering):
- Modality: Specification-first + Socratic dialogue
- Approach: "Here's what good prompts achieve" → questions guide discovery → active creation
- Student role: Active creator discovering patterns

**Variation Achieved**: ✅ Different modality (not repeating Chapter 9 pattern)

### Within-Chapter Modality Variation

**Lessons 1-2**: Specification-first (show outcomes → analyze structure)
**Lessons 3-5**: Socratic + hands-on (questions → iteration → reflection)
**Lessons 6-7**: Design-focused (create templates → document criteria)
**Lesson 8**: Specification-driven (write spec → peer review)

**Variety**: ✅ Four distinct approaches across 8 lessons

---

## V. Cognitive Load Distribution

### Per-Lesson Concept Count

| Lesson | Tier | Concepts | Load Assessment |
|--------|------|----------|-----------------|
| L1 | A2 | 5 | ✅ Within A2 limit (5-7) |
| L2 | A2 | 6 | ✅ Within A2 limit (5-7) |
| L3 | A2→B1 | 7 | ✅ Within B1 limit (7-10) |
| L4 | B1 | 7 | ✅ Within B1 limit (7-10) |
| L5 | B1 | 7 | ✅ Within B1 limit (7-10) |
| L6 | B1 | 8 | ✅ Within B1 limit (7-10) |
| L7 | B1 | 7 | ✅ Within B1 limit (7-10) |
| L8 | B1 | 0 new (synthesis) | ✅ No overload (composition only) |

**Progressive Load Increase**: ✅ 5 → 6 → 7 → 8 (gradual)
**Tier Alignment**: ✅ A2 (5-7), B1 (7-10)

### Concept Chunking Effectiveness

**Related concepts grouped**:
- L1-2: Prompt foundations (chunk together naturally)
- L3-5: AI collaboration patterns (build on each other)
- L6-7: Template creation (sequential dependency)

**Cognitive load reduction**: ✅ Chunking reduces working memory demands

---

## VI. Platform Coverage Strategy

### Claude Code vs Gemini CLI

**Approach**: Minimal differentiation (both use natural language prompts)

**Lesson Integration**:
- Examples use both tools interchangeably
- Syntax differences noted when relevant (file references: `@filename` in Gemini)
- Focus on PROMPT CONTENT (not tool-specific features)

**NOTE**: Detailed tool comparison belongs in Chapter 11 (context engineering)
- Context window sizes: Chapter 11 topic
- Session management: Chapter 11 topic
- Tool selection criteria: Chapter 11 topic

**Chapter 10 Scope**: Prompt text content only (universal across tools)

---

## VII. Constitutional Compliance Checklist

### 4-Stage Framework

- [x] **Stage 1** (L1-2): Manual foundation, ZERO AI tool usage
- [x] **Stage 2** (L3-5): AI collaboration, THREE ROLES demonstrated in ALL lessons
- [x] **Stage 3** (L6-7): Intelligence design, reusable templates created
- [x] **Stage 4** (L8): Spec-driven capstone, orchestrates accumulated intelligence

### Principle Compliance

- [x] **Principle 1** (Specification Primacy): L1-2 show WHAT before HOW, L4 teaches spec-first
- [x] **Principle 2** (Progressive Complexity): A2 (5-7) → B1 (7-10) progression
- [x] **Principle 3** (Factual Accuracy): Jake Heller framework cited, unverified claims removed
- [x] **Principle 6** (Anti-Convergence): Spec-first + Socratic vs Chapter 9 direct teaching
- [x] **Principle 7** (Minimal Content): ZERO context engineering (all in Chapter 11)

### Student-Facing Language Protocol

- [x] NO meta-commentary in lessons ("AI is teaching you patterns")
- [x] NO scaffolding exposure ("This is Stage 2")
- [x] NO framework labels in student text ("## Three Roles in Action")
- [x] Students EXPERIENCE Three Roles through narrative, not labels

### Lesson Ending Protocol

- [x] ALL lessons end with ONLY "Try With AI" section
- [x] NO "What's Next", "Key Takeaways", "Summary", standalone "Safety Note"

---

## VIII. Success Criteria

### Plan Succeeds When

- [x] Lesson count derived from concept density (8 lessons, not arbitrary)
- [x] Pedagogical arc follows Foundation → Application → Integration → Mastery
- [x] Teaching modality varies from Chapter 9 (specification-first + Socratic)
- [x] Stage progression explicit (Manual → AI Collab → Intelligence → Spec-Driven)
- [x] Cognitive load managed (A2: 5-7, B1: 7-10 concepts per lesson)
- [x] Intelligence accumulation demonstrated (templates in L6-7, orchestrated in L8)
- [x] Zero overlap with Chapter 11 (no context engineering content)

### Plan Fails When

- [ ] Lesson count arbitrary (not justified by concept density)
- [ ] Teaching modality same as Chapter 9 (direct teaching)
- [ ] Stage progression not followed (skipping stages or wrong order)
- [ ] Cognitive load violated (too many concepts per lesson)
- [ ] Context engineering concepts appear (violates separation)
- [ ] No intelligence accumulation (templates not reusable)

---

## IX. Next Steps: Task Breakdown

**After plan approval, tasks.md will define**:
1. One task per lesson (L1-L8)
2. Deliverables per task (markdown files, exercises, templates)
3. Validation criteria per lesson
4. Anti-pattern checks (prevent context engineering content)
5. Constitutional compliance validation per lesson

**Implementation approach**:
- Implement lessons sequentially (L1 → L8)
- Validate each lesson before proceeding to next
- Apply anti-pattern detection continuously
- Use content-implementer agent or direct implementation

---

**Plan Version**: 1.0
**Ready for**: User approval and task breakdown phase
