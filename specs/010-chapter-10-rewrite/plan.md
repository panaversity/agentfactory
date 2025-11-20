# Chapter 10 Rewrite Plan: Prompt Engineering for AI-Driven Development

**Specification**: specs/010-chapter-10-rewrite/spec.md v2.0 (Collaboration-Focused)
**Created**: 2025-11-18
**Revised**: 2025-11-18 (Alignment with approved spec v2.0)
**Constitutional Version**: 6.0.1
**Lesson Count**: 8 lessons (derived from concept density analysis)

---

## CRITICAL CHANGE: Collaboration-Focused Approach

**OLD APPROACH (v1.0)**: Specification writing as PM skill
**NEW APPROACH (v2.0)**: Clear communication for AI collaboration

**Rationale**: Part 3, A2-B1 students have NO programming experience yet. Must teach collaboration skills (Layer 2), NOT specification writing (Layer 4).

---

## I. Pedagogical Architecture

### Core Thesis

**"Clear prompts get better AI responses. This chapter teaches you how to communicate your intent effectively so AI can help you accomplish tasks."**

NOT teaching:
- ❌ Specification writing for AI execution
- ❌ Professional PM frameworks
- ❌ Jake Heller's 60%→97% as primary framework (too advanced for A2)

Teaching:
- ✅ Communication skill: How do I ask AI clearly?
- ✅ Collaboration skill: How do I work WITH AI iteratively?
- ✅ Pattern recognition: When should I use similar prompts?
- ✅ Validation skill: How do I check AI's work?

### Concept Density Analysis

**Total Concepts**: 42 concepts across 8 lessons

**Chunking Analysis**:
1. **Manual Foundation (L1-L3)**: 12 concepts → 3 lessons (4-5 concepts each, A2 tier)
2. **AI Collaboration (L4-L6)**: 17 concepts → 3 lessons (5-6 concepts each, A2→B1)
3. **Intelligence Design (L7-L8)**: 13 concepts → 2 lessons (7 concepts L7, integration L8, B1 tier)

**Lesson Count Justification**: 8 lessons
- Derived from A2 (5-7 concepts) → B1 (7-10 concepts) progression
- NOT arbitrary (not forcing pattern)
- Respects pedagogical phases (Manual → Collaboration → Intelligence)

### Layer Progression (4-Layer Teaching Method)

**LAYER 1: Manual Foundation (L1-L3)** — Build mental models WITHOUT AI
- **Purpose**: Understand what makes prompts effective BEFORE using AI
- **Cognitive Load**: A2 tier (5-7 concepts per lesson)
- **Teaching Modality**: Comparison-based (vague vs. clear examples)
- **Outcome**: Students can identify prompt quality manually

**LAYER 2: AI Collaboration (L4-L6)** — Partner with AI using Three Roles
- **Purpose**: Experience Teacher/Student/Co-Worker dynamics naturally
- **Cognitive Load**: A2→B1 transition (5-7 concepts per lesson)
- **Teaching Modality**: Experiential (hands-on iteration, reflection)
- **Outcome**: Students collaborate with AI bidirectionally

**LAYER 3: Intelligence Design (L7-L8)** — Create reusable patterns
- **Purpose**: Recognize recurring tasks, create prompt templates
- **Cognitive Load**: B1 tier (7-10 concepts per lesson)
- **Teaching Modality**: Design-focused (template creation, decision guides)
- **Outcome**: Personal prompt toolkit with usage criteria

### Teaching Modality Strategy (Anti-Convergence)

**Chapter 9 Modality**: Direct teaching (lecture-style markdown tutorial)

**Chapter 10 Modality** (MUST vary):
- **Experiential collaboration**: Students practice with AI, discover patterns through iteration
- **Comparison-based**: Show vague vs. clear prompts, analyze differences
- **Reflection-driven**: "What worked? What didn't? Why?"
- **NOT**: Lecture-style (avoid Chapter 9 pattern)

**Per-Lesson Modality Assignment**:
- **L1-3**: Comparison + manual practice (analyze examples WITHOUT AI)
- **L4-6**: Experiential + hands-on (practice WITH AI, reflect on process)
- **L7-8**: Design + synthesis (create templates, build toolkit)

---

## II. Lesson-by-Lesson Breakdown

### Lesson 1: Why Prompting Matters

**Layer**: 1 (Manual Foundation)
**Duration**: 30 minutes
**Proficiency**: A2
**Concepts**: 3

**Learning Objectives**:
- LO-001: Distinguish vague from specific prompts
- Understand: Vague prompts → Poor responses / Specific prompts → Better responses
- Mental model: AI as collaborator, not magic oracle

**Concept List**:
1. Vague prompts produce poor AI responses (comparison)
2. Specific prompts produce better AI responses (before/after)
3. AI as collaborator requiring clear communication

**Teaching Modality**: Comparison-based
- Show: 5 prompt pairs (vague vs. specific)
- Analyze: Why does specific version work better?
- Practice: Improve 3 vague prompts WITHOUT AI (manual exercise)

**Practice Vehicle**: Bash command explanations
- Vague: "help with ls"
- Specific: "explain ls -la output showing file permissions and sizes"

**Prerequisites**:
- Chapter 7: Bash basics
- Chapter 9: Markdown syntax

**Deliverable**: Improved prompts (manual exercise, documented in markdown)

**Ending Section**: ONLY "Try With AI" (test improved prompts, compare results)

---

### Lesson 2: Basic Prompt Structure

**Layer**: 1 (Manual Foundation)
**Duration**: 35 minutes
**Proficiency**: A2
**Concepts**: 4

**Learning Objectives**:
- LO-002: Structure prompts using Task + Context + Format
- Apply: Three-element pattern to Bash, Git, Markdown tasks

**Concept List**:
1. Task element: WHAT you want (verb-first: explain, create, debug, optimize)
2. Context element: Background AI needs (project type, experience level)
3. Format element: HOW you want answer (code block, bullet points, table)
4. Three-element pattern combines all (Task + Context + Format)

**Teaching Modality**: Comparison + structure analysis
- Show: Unstructured vs. structured prompts
- Identify: Task, Context, Format elements in examples
- Practice: Write 3 structured prompts (manual, no AI)

**Practice Vehicle**: Git explanations
- Task: "Explain git rebase"
- Context: "assuming I understand merge"
- Format: "in 3 bullet points with examples"

**Prerequisites**:
- Lesson 1: Why prompts matter
- Chapter 8: Git basics

**Deliverable**: 3 structured prompts (Bash, Git, Markdown tasks)

**Ending Section**: ONLY "Try With AI" (test prompts, evaluate responses)

---

### Lesson 3: Adding Examples and Constraints

**Layer**: 1 (Manual Foundation)
**Duration**: 40 minutes
**Proficiency**: A2
**Concepts**: 5

**Learning Objectives**:
- LO-003: Enhance prompts with examples and constraints
- Apply: Add 2 constraints + 1 example to basic prompts

**Concept List**:
1. Examples show desired output style (show don't tell)
2. Constraints limit scope (length, complexity, language level)
3. Multiple examples improve consistency
4. Constraints prevent unwanted behavior (security, complexity)
5. Iterative refinement pattern (add constraints progressively)

**Teaching Modality**: Enhancement practice
- Show: Basic vs. enhanced prompts
- Identify: What constraints/examples add
- Practice: Enhance L2 prompts with constraints/examples

**Practice Vehicle**: Markdown documentation
- Basic: "Create markdown doc for project README"
- Enhanced: "Create markdown README similar to [example], max 200 words, beginner-friendly language, include installation and usage sections"

**Prerequisites**:
- Lessons 1-2: Prompt foundations
- Chapter 9: Markdown syntax

**Deliverable**: Enhanced prompts (basic prompt + examples/constraints added)

**Ending Section**: ONLY "Try With AI" (compare basic vs. enhanced results)

---

### Lesson 4: Iterative Refinement

**Layer**: 2 (AI Collaboration)
**Duration**: 40 minutes
**Proficiency**: A2→B1 transition
**Concepts**: 5

**Learning Objectives**:
- LO-004: Demonstrate Three Roles in iteration (experienced, not labeled)
- Apply: 3-iteration refinement loop (initial → refine → refine → final)

**Concept List**:
1. First prompt rarely perfect (iteration mindset)
2. AI as Teacher: Suggests improvements you didn't know
3. AI as Student: You teach AI your specific context
4. AI as Co-Worker: Iteration produces better result than either alone
5. Convergence loop: Human + AI converge on solution

**Teaching Modality**: Experiential collaboration
- Students: Start with initial prompt
- AI: Responds (may be generic or miss context)
- Students: Refine based on gaps
- Iterate: Until output meets needs
- Reflect: What improved through iteration?

**Three Roles Experienced** (NOT labeled as framework):
- Narrative shows AI suggesting patterns (Teacher role naturally)
- Narrative shows student correcting AI (Student role naturally)
- Narrative shows convergence through iteration (Co-Worker naturally)

**Practice Vehicle**: Bash script debugging
- Iteration 1: "Debug this script" → Generic response
- Iteration 2: "Debug this script; it fails with 'permission denied' on line 5" → Specific fix
- Iteration 3: "Explain WHY this permission issue happens" → Understanding

**Prerequisites**:
- Lessons 1-3: Manual foundations
- Chapter 7: Bash scripting

**Deliverable**: Iteration log (3 refinement cycles documented with reflection)

**Jake Heller Reference**: Sidebar in lesson
- "Expert Insight: Jake Heller (Casetext founder) spent weeks refining prompts from 60% → 97% accuracy through iteration. This lesson teaches that same iterative mindset."
- NOT taught as framework to apply

**Ending Section**: ONLY "Try With AI" (full iteration session with reflection prompts)

---

### Lesson 5: Question-Driven Development

**Layer**: 2 (AI Collaboration)
**Duration**: 45 minutes
**Proficiency**: B1
**Concepts**: 6

**Learning Objectives**:
- LO-005: Apply Question-Driven Development (prompt AI to ask questions first)
- Understand: AI questions reveal assumptions and improve tailored solutions

**Concept List**:
1. AI can ask YOU questions before answering
2. Questions reveal assumptions and gaps in your request
3. Your answers provide context AI needs
4. Question-driven produces tailored solutions (vs. generic)
5. Comparison: Direct prompt vs. question-driven approach
6. When to use: Complex tasks with multiple valid approaches

**Teaching Modality**: Experiential + comparison
- Direct approach: "Explain git workflows" → Generic answer
- QDD approach: "Before explaining git workflows, ask me 5 questions" → AI asks about team size, project type, release cadence → Tailored answer
- Students: Compare results, reflect on difference

**Practice Vehicle**: Git workflow design
- QDD prompt: "Before explaining git workflows, ask me 5 questions about my experience and project needs"
- AI: Asks clarifying questions
- Student: Answers thoughtfully
- AI: Provides tailored explanation based on answers

**Prerequisites**:
- Lesson 4: Iterative refinement
- Chapter 8: Git workflows

**Deliverable**: QDD session log (questions asked, answers given, comparison with direct approach)

**Ending Section**: ONLY "Try With AI" (QDD practice on complex task)

---

### Lesson 6: Validating AI Outputs

**Layer**: 2 (AI Collaboration)
**Duration**: 40 minutes
**Proficiency**: B1
**Concepts**: 6

**Learning Objectives**:
- LO-006: Validate AI outputs using checklist (never trust blindly)
- Apply: Read → Understand → Test → Question unfamiliar parts

**Concept List**:
1. Never trust AI blindly (validation mindset)
2. Validation checklist: Read → Understand → Test → Question
3. Red flags: Overly complex, missing error handling, unexplained "magic"
4. Ask AI to explain unfamiliar parts (learning opportunity)
5. Test outputs in safe environment before production use
6. Iterate when validation fails (refine prompt, request fixes)

**Teaching Modality**: Critical analysis + experiential
- Present: AI-generated Bash script with issues
- Students: Apply validation checklist
- Identify: Red flags (hardcoded paths, no error handling)
- Prompt: "Why is this path hardcoded? How can I make it configurable?"
- AI: Explains and provides improved version

**Practice Vehicle**: Bash script validation
- AI generates backup script with hardcoded paths
- Student identifies: Path hardcoded, no error handling, no user feedback
- Student prompts: Explain issues, request improvements
- Validate: Improved version meets criteria

**Prerequisites**:
- Lesson 4-5: Iteration and QDD
- Chapter 7: Bash scripting

**Deliverable**: Validation report (issues identified, prompts for fixes, improved output)

**Ending Section**: ONLY "Try With AI" (validate and improve AI-generated solution)

---

### Lesson 7: Building Your Prompt Library

**Layer**: 3 (Intelligence Design)
**Duration**: 45 minutes
**Proficiency**: B1
**Concepts**: 7

**Learning Objectives**:
- LO-007: Create 3+ reusable prompt templates with placeholders
- Design: Templates for recurring tasks (pattern recognition)

**Concept List**:
1. Recognize recurring tasks (pattern recognition)
2. Extract reusable structure (template creation)
3. Placeholders for variables (generalization: [VARIABLE])
4. Usage notes for context (when to use this template)
5. Template categories: Explain, Debug, Create, Optimize
6. Version control for templates (iterate and improve)
7. Test templates on new scenarios (validation)

**Teaching Modality**: Design-focused
- Identify: Tasks you do repeatedly (Bash debugging, Git commits, Markdown docs)
- Extract: Common structure across instances
- Create: Template with placeholders
- Document: When to use, how to customize
- Test: Apply template to new scenario

**Practice Vehicle**: Template creation for common tasks
- **Git commit template**: "Write commit message for [FEATURE] that [ACTION], following [STYLE]"
- **Bash debug template**: "Debug Bash script [FILENAME] failing with [ERROR] at [LINE]"
- **Markdown doc template**: "Create markdown doc for [PROJECT] including [SECTIONS]"

**Prerequisites**:
- All Lessons 1-6: Full prompt engineering foundation
- Experience with Bash, Git, Markdown (Chapters 7-9)

**Deliverable**: 3 templates (markdown files with placeholders + usage notes)

**Ending Section**: ONLY "Try With AI" (test templates on new scenarios, refine based on results)

---

### Lesson 8: Capstone - Your First Prompt Toolkit

**Layer**: 3 (Intelligence Design) + Capstone Integration
**Duration**: 60 minutes
**Proficiency**: B1
**Concepts**: Integration (no new concepts)

**Learning Objectives**:
- LO-008: Build comprehensive prompt toolkit (5-7 templates + decision guide)
- Demonstrate: Template selection criteria (WHEN to use each)

**Concept List** (Integration only):
- Orchestrates: All skills from L1-L7
- Integrates: Templates (L7) + Decision criteria + Validation (L6)
- Delivers: Personal prompt toolkit (portfolio piece)

**Teaching Modality**: Synthesis + design
- Audit: What tasks do YOU do repeatedly?
- Create: 5-7 templates covering your common tasks
- Document: Decision guide (task → template mapping)
- Validate: Test each template, peer review

**Practice Vehicle**: Complete toolkit creation

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

**Prerequisites**:
- ALL Lessons 1-7
- Templates from L7

**Deliverable**:
- Complete toolkit (markdown file, 5-7 templates)
- Decision guide (when to use each)
- Test results (validated on real scenarios)

**Assessment Criteria**:
- Toolkit has 5-7 templates with placeholders
- Each template has usage criteria
- Decision guide maps tasks to templates
- Peer can use toolkit without clarification

**Ending Section**: ONLY "Try With AI" (validate toolkit with AI feedback, test with peer)

---

## III. Intelligence Accumulation Map

### Skills Created (Layer 3)

**Lesson 7 Deliverables**:
- 3+ prompt templates (reusable patterns)
- Template structure understanding
- Pattern recognition skill

**Lesson 8 Deliverables**:
- Complete prompt toolkit (5-7 templates)
- Decision guide (template selection)
- Validation framework

### Reusability Across Book

**Templates Apply To**:
- **Part 4 (Python)**: Debugging Python scripts, explaining syntax, refactoring code
- **Part 5 (Spec-Driven)**: Writing specifications, creating plans
- **Part 6+ (Advanced)**: Domain-specific adaptations

**Decision Framework Applies To**:
- All future AI collaboration
- Professional workflows
- Team collaboration (shareable templates)

---

## IV. Anti-Convergence Validation

### Teaching Modality Comparison

**Chapter 9** (Markdown):
- Modality: Direct teaching (lecture-style syntax tutorial)
- Approach: "Here's markdown syntax" → demonstrations → practice exercises
- Student role: Passive learner following instructions

**Chapter 10** (Prompt Engineering):
- Modality: Experiential collaboration (hands-on practice with AI)
- Approach: "Try this prompt" → compare results → discover what works → reflect
- Student role: Active collaborator discovering patterns through iteration

**Variation Achieved**: ✅ Different modality (experiential vs. direct teaching)

### Within-Chapter Modality Variation

**L1-3**: Comparison-based (vague vs. clear, analyze manually)
**L4-6**: Experiential (hands-on with AI, reflection after iterations)
**L7-8**: Design-focused (create templates, build toolkit)

**Variety**: ✅ Three distinct approaches across 8 lessons

---

## V. Cognitive Load Distribution

### Per-Lesson Concept Count

| Lesson | Tier | Concepts | Load Assessment |
|--------|------|----------|-----------------|
| L1 | A2 | 3 | ✅ Within A2 limit (5-7) |
| L2 | A2 | 4 | ✅ Within A2 limit (5-7) |
| L3 | A2 | 5 | ✅ Within A2 limit (5-7) |
| L4 | A2→B1 | 5 | ✅ Within B1 limit (7-10) |
| L5 | B1 | 6 | ✅ Within B1 limit (7-10) |
| L6 | B1 | 6 | ✅ Within B1 limit (7-10) |
| L7 | B1 | 7 | ✅ Within B1 limit (7-10) |
| L8 | B1 | 0 new (integration) | ✅ No overload (synthesis only) |

**Total Concepts**: 36 new + 6 integration = 42 total
**Progressive Load**: ✅ 3 → 4 → 5 → 5 → 6 → 6 → 7 (gradual increase)
**Tier Alignment**: ✅ A2 (3-5), B1 (5-7)

### Concept Chunking Effectiveness

**Natural groupings**:
- L1-3: Prompt foundations (structure, clarity, enhancement)
- L4-6: AI collaboration (iteration, questions, validation)
- L7-8: Intelligence design (templates, toolkit)

**Cognitive load reduction**: ✅ Chunking reduces working memory demands

---

## VI. Platform Coverage Strategy

### Claude Code vs Gemini CLI

**Approach**: Minimal differentiation (both use natural language prompts)

**Chapter 10 Focus**: Prompt CONTENT (what you say)
- Task + Context + Format structure
- Examples and constraints
- Iteration patterns
- Template design

**Chapter 11 Focus**: Context management (what AI knows)
- Context window sizes
- Session management
- Memory files
- Tool selection criteria

**Lesson Integration**:
- Examples use "Claude Code or Gemini CLI" interchangeably
- Focus on prompt text (universal across tools)
- Syntax differences noted only when relevant

---

## VII. Constitutional Compliance Checklist

### 4-Layer Teaching Method

- [x] **Layer 1** (L1-3): Manual foundation, ZERO AI tool usage until "Try With AI"
- [x] **Layer 2** (L4-6): AI collaboration, Three Roles experienced naturally (NOT labeled)
- [x] **Layer 3** (L7-8): Intelligence design, reusable templates created
- [x] Capstone (L8): Integration of all skills (toolkit building)

### Principle Compliance

- [x] **Principle 1** (Specification Primacy): Applied to pedagogy (show outcomes first), NOT content (collaboration, not specification writing)
- [x] **Principle 2** (Progressive Complexity): A2 (3-5) → B1 (5-7) progression
- [x] **Principle 3** (Factual Accuracy): Jake Heller referenced as sidebar, Anthropic/Google/OpenAI docs cited
- [x] **Principle 6** (Anti-Convergence): Experiential collaboration vs Chapter 9 direct teaching
- [x] **Principle 7** (Minimal Content): ZERO context engineering (all in Chapter 11)

### Student-Facing Language Protocol

- [x] NO meta-commentary in lessons ("Notice how AI teaches you patterns")
- [x] NO scaffolding exposure ("This is Layer 2, so we use Three Roles")
- [x] NO framework labels ("## Three Roles Framework")
- [x] Students EXPERIENCE Three Roles through natural iteration narrative

### Lesson Ending Protocol

- [x] ALL lessons end with ONLY "Try With AI" section
- [x] NO "What's Next", "Key Takeaways", "Summary", standalone "Safety Note"

---

## VIII. Separation from Chapter 11 (Context Engineering)

### Chapter 10 Scope (WHAT you SAY)

✅ Included:
- Clear prompt writing (Task + Context + Format)
- Iteration and refinement
- Question-driven collaboration
- Prompt templates for common tasks
- Validation of AI outputs

### Chapter 11 Scope (WHAT AI KNOWS)

❌ Excluded from Chapter 10:
- Context windows and token limits
- Progressive loading strategies
- Memory files (CLAUDE.md, architecture.md)
- Session management
- Context compression techniques

---

## IX. Success Criteria

### Plan Succeeds When

- [x] 8-lesson structure derived from concept density (A2→B1 progression)
- [x] Collaboration-focused approach (NOT specification writing)
- [x] Layer progression: Manual → Collaboration → Intelligence
- [x] Teaching modality varies from Chapter 9 (experiential vs. direct)
- [x] Three Roles experienced naturally in L4-6 (NOT labeled as framework)
- [x] Jake Heller referenced as sidebar (NOT primary framework)
- [x] Zero context engineering content (clean separation with Chapter 11)
- [x] Practice uses Bash/Git/Markdown ONLY (NO Python)

### Plan Fails When

- [ ] Teaching specification writing (wrong layer for A2-B1)
- [ ] Using professional frameworks as primary pedagogy
- [ ] Context engineering concepts appear (violates Chapter 11 separation)
- [ ] Python code examples used (violates Part 3 constraints)
- [ ] Meta-commentary exposes scaffolding (violates protocol)
- [ ] Three Roles taught as framework instead of experienced naturally

---

## X. Next Steps: Task Breakdown

**After plan approval, tasks.md will define**:
1. Eight implementation tasks (one per lesson: L1-L8)
2. Clear deliverables per task (lesson markdown files)
3. Validation criteria per lesson (constitutional compliance, factual accuracy)
4. Anti-pattern checks (prevent context engineering, specification writing language)
5. Three Roles verification (L4-6 demonstrate naturally, not labeled)

**Implementation approach**:
- Create lessons sequentially (L1 → L8)
- Validate each lesson before proceeding
- Apply anti-pattern detection (no context engineering, no PM language)
- Ensure practice uses Bash/Git/Markdown ONLY

---

**Plan Version**: 2.0 (Collaboration-Focused, aligned with spec v2.0)
**Ready for**: User approval and task breakdown phase
