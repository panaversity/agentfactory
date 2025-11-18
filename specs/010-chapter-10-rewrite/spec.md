# Chapter 10 Rewrite Specification: Prompt Engineering for AI-Driven Development

**Feature ID**: 010-chapter-10-rewrite
**Created**: 2025-11-18
**Status**: Specification Phase
**Constitutional Version**: 6.0.1
**Complexity Tier**: B1 (Intermediate)

---

## I. Evals: What Success Looks Like

### Learning Outcomes (Measurable)

**Foundation Level (Stage 1: Manual)**
- **LO-001**: Student can explain prompt engineering as "specification skill" using Jake Heller's framework ("60% → 97% accuracy through iteration"), NOT as "getting AI to do things faster"
- **LO-002**: Student can analyze 5 prompt pairs (vague vs specific) and identify WHY specific prompts produce better results WITHOUT AI assistance
- **LO-003**: Student can manually structure prompts using "Intent → Constraints → Success Criteria" pattern on paper/markdown

**Application Level (Stage 2: AI Collaboration)**
- **LO-004**: Student demonstrates THREE ROLES framework in prompt refinement session:
  - AI as Teacher: AI suggests prompt improvements student didn't know
  - AI as Student: Student teaches AI project-specific constraints
  - AI as Co-Worker: Iterative convergence produces better prompt than either had initially
- **LO-005**: Student writes specification-first prompts (defines WHAT success looks like BEFORE asking AI to generate code)
- **LO-006**: Student applies Question-Driven Development (prompts AI to ask 5+ clarifying questions before generating solution)

**Integration Level (Stage 3: Intelligence Design)**
- **LO-007**: Student creates 3+ reusable prompt templates for common tasks (new feature, bug fix, refactoring, testing)
- **LO-008**: Student documents WHEN to use each template (decision framework, not just library)

**Mastery Level (Stage 4: Spec-Driven)**
- **LO-009**: Student writes complete prompt template library specification that peer can use WITHOUT asking clarifying questions

### Success Metrics

**Quantitative**:
- 85%+ students can distinguish prompt engineering (communication) from context engineering (information management)
- 90%+ students can identify Stage 1 vs Stage 2 prompting situations
- 75%+ students create working prompt templates in capstone

**Qualitative**:
- Students explain prompts as "specifications for AI execution" (professional framing)
- Students avoid "AI will do my work" mindset, embrace "AI executes what I specify" mindset
- Students reference Jake Heller framework ("iteration gets 60% → 97%") when discussing prompt quality

### Assessment Methods

**Stage 1 (Manual)**:
- Matching exercise: Vague prompt → Poor outcome; Specific prompt → Good outcome
- Analysis task: Given 3 prompts, identify which achieves specification-first pattern

**Stage 2 (AI Collaboration)**:
- Live demonstration: Student runs prompt iteration session showing all Three Roles
- Reflection: Student writes 200-word explanation of how AI collaboration improved their prompt

**Stage 3 (Intelligence Design)**:
- Artifact: Student submits prompt template library (3+ templates with usage criteria)
- Peer review: Another student uses templates without needing clarification

**Stage 4 (Spec-Driven)**:
- Capstone specification: Reviewer evaluates spec completeness (Intent, Constraints, Success Criteria, Non-Goals, Acceptance Tests)

---

## II. Intent: What We're Building

### Core Thesis

**Prompt engineering is specification writing for AI execution.**

In the agentic era, the developer who can specify intent clearly outperforms the developer who types code fastest. Prompt engineering is NOT about "getting AI to work faster"—it's about **thinking like a product manager** who writes requirements so precise that any implementation (human or AI) will succeed.

### What This Chapter Teaches

**Core Skill**: Prompt engineering as professional specification skill

**Pedagogical Approach**: Specification-first + Socratic dialogue (anti-convergence from Chapter 9 direct teaching)

**Teaching Sequence**:
1. **Stage 1 (Manual)**: Analyze prompt structure manually, no AI assistance
2. **Stage 2 (AI Collaboration)**: Iterate prompts with AI showing all Three Roles
3. **Stage 3 (Intelligence Design)**: Create reusable prompt templates (2-4 decision points each)
4. **Stage 4 (Spec-Driven)**: Orchestrate prompt template library via specification

### Separation from Chapter 11 (Context Engineering)

**Chapter 10 Scope** (Prompt Engineering):
- WHAT you say to AI (communication skill)
- Prompt structure (Intent, Constraints, Success Criteria)
- Iteration and refinement (Jake Heller's 60% → 97% framework)
- Specification-first thinking (define success BEFORE prompting)
- Question-Driven Development (AI asks clarifying questions)
- Prompt templates for common tasks

**Chapter 11 Scope** (Context Engineering - NOT in Chapter 10):
- WHAT AI knows when you prompt (information management)
- Context windows and token limits
- Progressive loading strategies
- Context compression and isolation
- Memory files (CLAUDE.md, architecture.md)
- Session management across days/weeks

**Prohibited Content in Chapter 10**:
- ❌ Context window explanations (belongs in Chapter 11)
- ❌ Token counting (belongs in Chapter 11)
- ❌ Memory file architecture (belongs in Chapter 11)
- ❌ Progressive loading (belongs in Chapter 11)

### Research Foundation

**Verified Claims (with sources)**:

**Jake Heller Framework** (Casetext/CoCounsel, $650M exit by Thomson Reuters):
- Source: Y Combinator talk, 2024 (https://www.youtube.com/watch?v=l0h3nAW13ao)
- Claim: "Spend weeks tweaking prompts to get from 60% accuracy to 97%+. Most people quit too early." [20:03]
- Application: Prompt engineering is iterative refinement, not one-shot attempts
- Claim: "Define 'what good looks like' for every step. Create objective tests." [15:27-18:16]
- Application: Specification-first prompting (success criteria before prompt text)

**Nine Pillars of AIDD** (Chapter 4 reference):
- Framework exists in Chapter 4 (NOT "8-element framework")
- Students have prerequisite knowledge of AIDD philosophy

**Removed Unverified Claims**:
- ❌ "55% more productive" → No authoritative source (REMOVED)
- ❌ "8-element AIDD framework" → Doesn't exist; confused with 9 Pillars (REMOVED)

### Audience Tier: A2 → B1 Progression

**Part 3 Constraints**:
- Students have NOT learned Python (Python starts Part 4, Chapter 12+)
- Students HAVE completed: Bash (Ch7), Git (Ch8), Markdown (Ch9)
- Students understand: Command-line interfaces, file operations, markdown formatting

**Cognitive Load Limits**:
- **Lessons 1-2** (A2 Foundation): 5-7 concepts per section
- **Lessons 3-5** (A2→B1 Application): 7 concepts per section
- **Lessons 6-8** (B1 Integration/Mastery): 7-10 concepts per section

**Practice Vehicles** (NO Python code):
- Bash commands (prompt AI to explain/debug/optimize shell scripts)
- Git operations (prompt AI to write commit messages, explain git commands)
- Markdown documents (prompt AI to create structured documentation)
- Conceptual exercises (analyze prompts, identify patterns)

---

## III. Constraints

### Constitutional Compliance

**Mandatory Frameworks**:
- Section IIa: 4-Stage Teaching Framework (Manual → AI Collab → Intelligence → Spec-Driven)
- Principle 2: Progressive Complexity (A2: 5-7 concepts, B1: 7-10 concepts)
- Principle 3: Factual Accuracy (all claims verified, sources cited)
- Principle 6: Anti-Convergence (vary from Chapter 9 direct teaching → use Specification-first + Socratic)
- Principle 7: Minimal Content (ONLY prompt engineering, NO context engineering)

**Student-Facing Language Protocol**:
- ❌ NO meta-commentary: "What to notice: AI is teaching you patterns"
- ❌ NO scaffolding exposure: "This is Layer 2, so we'll use AI collaboration"
- ❌ NO framework labels in student text: "## Three Roles in Action"
- ✅ Students EXPERIENCE pedagogy, don't STUDY it

**Lesson Ending Protocol**:
- ✅ ONLY final section: "Try With AI"
- ❌ NO "What's Next", "Key Takeaways", "Summary", standalone "Safety Note"

### Technical Constraints

**No Python Code**:
- Students have NOT learned Python yet (Part 4 starts at Chapter 12)
- Practice vehicles: Bash, Git, Markdown, conceptual analysis ONLY
- When showing "code" examples, use Bash scripts or pseudocode with clear labels

**Platform Coverage**:
- Claude Code examples (Anthropic's official tool)
- Gemini CLI examples (Google's official tool)
- Minimal syntax differences (both use natural language prompts)
- NOTE: Detailed tool comparison belongs in Chapter 11 (context engineering)

**Factual Accuracy**:
- ALL claims must have authoritative sources
- Jake Heller framework: Cited with YouTube timestamp
- NO hallucinated frameworks or statistics
- If claim cannot be verified → REMOVE

### Pedagogical Constraints

**Teaching Modality** (Anti-Convergence):
- Chapter 9 used: Direct teaching (lecture-style markdown tutorial)
- Chapter 10 MUST use: Specification-first + Socratic dialogue
- Specification-first: Show WHAT prompts achieve BEFORE teaching HOW to write them
- Socratic: Questions guide students to discover prompt patterns

**Three Roles Framework** (Stage 2 Lessons):
- MUST demonstrate all three roles: AI as Teacher, Student, Co-Worker
- MUST show bidirectional learning (not passive AI tool use)
- MUST include convergence loop (iteration improves both human and AI understanding)

**Stage Progression** (Non-Negotiable):
- Lessons 1-2: Stage 1 (Manual) — ZERO AI tool usage, manual analysis only
- Lessons 3-5: Stage 2 (AI Collaboration) — Three Roles demonstrated
- Lessons 6-7: Stage 3 (Intelligence Design) — Create reusable prompt templates
- Lesson 8: Stage 4 (Spec-Driven) — Capstone specification (NO implementation code)

### Content Structure Constraints

**Lesson Count**:
- NOT arbitrary (no forcing into 8 or 9 lessons)
- Derived from concept density analysis
- Expected range: 7-9 lessons based on prompt engineering scope

**Concept Chunking**:
- Related concepts group together (reduces cognitive load)
- Progressive disclosure: Simple → Complex within each lesson
- No concept overload (respect tier limits: A2: 5-7, B1: 7-10)

---

## IV. Non-Goals: What We're NOT Building

### Explicitly Excluded from Chapter 10

**Context Engineering** (Belongs in Chapter 11):
- Context window concepts (token limits, working memory)
- Token counting and estimation
- Progressive loading strategies
- Context compression and session restart
- Context isolation for parallel tasks
- Memory files (CLAUDE.md, architecture.md, decisions.md)
- Tool selection based on context capacity

**Python Programming** (Starts in Part 4):
- Python syntax or code examples
- Python-specific prompting patterns
- Python debugging with AI
- Python library usage

**Advanced AI Concepts** (Beyond Part 3 scope):
- Fine-tuning or model training
- RAG (Retrieval-Augmented Generation)
- Agentic workflows or multi-agent systems
- LLM internals or architecture

**Productivity Claims Without Evidence**:
- Generic "X% more productive" statistics
- Unverified frameworks or methodologies
- Anecdotal success stories without authoritative sources

### Why These Are Excluded

**Context Engineering Exclusion**:
- Chapter 11 already teaches this comprehensively
- Mixing prompt + context engineering creates cognitive overload
- Clean separation improves learning (one skill per chapter)

**Python Exclusion**:
- Students in Part 3 have NOT learned programming yet
- Premature code examples violate progressive complexity principle
- Part 4 (Chapters 12-29) teaches Python with AI collaboration

**Advanced Concepts Exclusion**:
- Beyond A2-B1 proficiency tier
- Requires prerequisite knowledge from later parts
- Violates minimal content principle (not essential to learning objectives)

---

## V. Acceptance Tests: How We Know It's Done

### Specification Completeness

- [ ] All 9 learning objectives (LO-001 through LO-009) have corresponding lesson content
- [ ] All claims cite authoritative sources (Jake Heller framework, official tool docs)
- [ ] Zero overlap with Chapter 11 content (no context engineering concepts)
- [ ] Zero Python code examples (practice vehicles use Bash/Git/Markdown only)

### Constitutional Compliance

- [ ] Stage 1 lessons (1-2) have ZERO AI tool usage (manual foundation)
- [ ] Stage 2 lessons (3-5) demonstrate ALL Three Roles (Teacher/Student/Co-Worker)
- [ ] Stage 3 lessons (6-7) create reusable prompt templates (intelligence design)
- [ ] Stage 4 lesson (8) is specification-only capstone (no implementation code)
- [ ] Teaching modality varies from Chapter 9 (specification-first + Socratic, not direct teaching)
- [ ] All lessons end with ONLY "Try With AI" section (no "What's Next", "Summary", etc.)
- [ ] Zero meta-commentary or scaffolding exposure in student-facing text

### Pedagogical Quality

- [ ] Concept density per section respects tier limits (A2: 5-7, B1: 7-10)
- [ ] Lesson count derived from concept analysis (not arbitrary 8 or 9)
- [ ] All practice exercises use appropriate vehicles (Bash, Git, Markdown)
- [ ] Jake Heller framework (60% → 97% iteration) integrated into teaching
- [ ] Specification-first pattern demonstrated throughout (WHAT before HOW)

### Factual Accuracy

- [ ] Jake Heller quotes verified with YouTube timestamps
- [ ] Claude Code capabilities cited from Anthropic documentation
- [ ] Gemini CLI capabilities cited from Google AI documentation
- [ ] Zero unverified productivity statistics
- [ ] Zero hallucinated frameworks (no "8-element AIDD")

### Validation Rigor

- [ ] Technical review: All prompts tested with Claude Code and Gemini CLI
- [ ] Pedagogical review: Stage progression validated
- [ ] Factual review: All sources verified
- [ ] Constitutional review: All principles applied correctly

---

## VI. Implementation Notes

### Lesson Structure Template (All Lessons)

```markdown
---
title: "[Lesson Title]"
chapter: 10
lesson: [N]
duration_minutes: [30-45]
---

# [Lesson Title]

[Opening: Why this matters, real-world connection]

## [Section 1: Core Concept Introduction]
[Specification-first: Show WHAT before HOW]

## [Section 2: Socratic Discovery]
[Questions guide students to patterns]

## [Section 3: Application]
[Practice with appropriate vehicles: Bash/Git/Markdown]

## Try With AI
[Active collaboration prompts]
[Self-reflection questions]
[Validation checks]
```

### Stage-Specific Requirements

**Stage 1 Lessons (1-2)**:
- NO AI tool usage sections
- Manual analysis exercises (paper, markdown documents)
- Build mental models for prompt evaluation

**Stage 2 Lessons (3-5)**:
- MUST include all Three Roles demonstration
- Iteration loops visible (initial → AI feedback → refinement → convergence)
- Reflection questions: "What did AI teach you?", "What did you teach AI?", "What emerged from iteration?"

**Stage 3 Lessons (6-7)**:
- Create prompt template deliverable (markdown file)
- Document usage criteria (decision framework)
- 2-4 decision points per template (matches skill complexity)

**Stage 4 Lesson (8)**:
- Specification ONLY (no implementation code)
- Orchestrate prompt templates from lessons 6-7
- Peer review validation (can another student use your spec?)

### Anti-Pattern Detection

**During Implementation, Prevent**:
- ❌ Mixing prompt and context engineering content
- ❌ Using Python code examples
- ❌ Unverified claims or statistics
- ❌ Direct teaching modality (violates anti-convergence)
- ❌ Meta-commentary exposure ("AI is teaching you patterns")
- ❌ Multiple final sections (only "Try With AI" allowed)

---

## VII. Success Criteria Summary

**This specification succeeds when**:

1. ✅ All 9 learning objectives mappable to lesson content
2. ✅ Zero overlap with Chapter 11 (context engineering cleanly separated)
3. ✅ All claims verified with authoritative sources
4. ✅ Constitutional compliance validated (4 stages, anti-convergence, minimal content)
5. ✅ Students can explain prompt engineering as "specification skill" (professional framing)
6. ✅ Students demonstrate Three Roles in AI collaboration (not passive tool use)
7. ✅ Students create reusable prompt templates (intelligence accumulation)
8. ✅ Capstone specification reviewable by peers (validation of mastery)

**This specification fails when**:

1. ❌ Context engineering concepts appear in lessons (violates separation)
2. ❌ Python code examples used (violates Part 3 constraints)
3. ❌ Unverified claims remain in content (violates factual accuracy)
4. ❌ Stage progression not followed (violates constitutional framework)
5. ❌ Teaching modality same as Chapter 9 (violates anti-convergence)
6. ❌ Meta-commentary exposes pedagogical scaffolding (violates student-facing protocol)

---

## VIII. Next Steps

**After Specification Approval**:

1. **Planning Phase** (plan.md):
   - Conduct concept density analysis
   - Derive lesson count from concepts (not arbitrary)
   - Map concepts to 4-stage progression
   - Define teaching modality per lesson
   - Create lesson-by-lesson breakdown

2. **Task Breakdown** (tasks.md):
   - One task per lesson (atomic units)
   - Include deliverables (markdown files, exercises, prompt templates)
   - Define validation criteria per lesson

3. **Implementation Phase**:
   - Use content-implementer agent or direct implementation
   - Follow specification exactly
   - Apply anti-pattern detection continuously

4. **Validation Phase**:
   - Technical review: Prompt testing
   - Pedagogical review: Stage progression check
   - Factual review: Source verification
   - Constitutional review: Compliance validation

---

**Specification Version**: 1.0
**Ready for**: User approval and planning phase
