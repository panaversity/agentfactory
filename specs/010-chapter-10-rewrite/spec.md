# Chapter 10 Redesign Specification: Prompt Engineering for AI-Driven Development

**Feature ID**: 010-chapter-10-rewrite
**Created**: 2025-11-18
**Revised**: 2025-11-18 (Collaboration-Focused Approach)
**Status**: Specification Phase
**Constitutional Version**: 6.0.1
**Complexity Tier**: A2 → B1 (Beginner to Intermediate Progression)

---

## Revision History

**v2.0 (2025-11-18)**: Complete redesign based on pedagogical analysis
- **Changed**: Core thesis from "specification writing" → "clear communication with AI"
- **Changed**: Teaching approach from professional/PM frameworks → beginner-friendly collaboration
- **Changed**: Removed Jake Heller 60%→97% as primary framework (too advanced for A2)
- **Changed**: 8-lesson structure aligned with Anthropic/Google/OpenAI official guidance
- **Rationale**: Chapter 10 is Part 3, A2-B1 students with NO programming experience. Must teach collaboration skills (Layer 2), NOT specification writing (Layer 4). Previous approach applied constitution's "Specification Primacy" literally without recognizing layer mismatch.

**v1.0 (2025-11-18)**: Original specification-focused approach (backed up as spec.md.backup-*)

---

## I. Context: What We Know About Students

### Prerequisites (Completed)
- **Chapter 7**: Bash basics (commands, file operations, scripting)
- **Chapter 8**: Git basics (commits, branches, push/pull)
- **Chapter 9**: Markdown syntax (headings, lists, code blocks, links)

### What Students DON'T Know
- ❌ **NO Python** (starts Part 4, Chapter 12)
- ❌ **NO programming** (variables, functions, classes)
- ❌ **NO specification writing** (professional skill, taught Part 5)
- ❌ **NO product management** (requirements, specs, acceptance criteria)

### Proficiency Level
- **Starting**: A2 (Can understand basic instructions, perform simple tasks)
- **Ending**: B1 (Can collaborate independently, create reusable patterns)
- **Cognitive Load**: A2 = 5-7 concepts/section, B1 = 7-10 concepts/section

---

## II. Intent: What We're Building

### Core Thesis (Revised)

**"Clear prompts get better AI responses. This chapter teaches you how to communicate your intent effectively so AI can help you accomplish tasks."**

This is NOT about:
- ❌ "Specification writing for AI execution" (too abstract for A2)
- ❌ "Thinking like a product manager" (wrong mental model)
- ❌ Professional prompt engineering (Jake Heller's 60%→97% is C1-C2 skill)

This IS about:
- ✅ **Communication skill**: How do I ask AI clearly?
- ✅ **Collaboration skill**: How do I work WITH AI iteratively?
- ✅ **Pattern recognition**: When should I use similar prompts?
- ✅ **Validation skill**: How do I check AI's work?

### Learning Journey (8 Lessons)

**LAYER 1: Manual Foundation (L1-L3)** — Build mental models WITHOUT AI
- L1: Why Prompting Matters (vague vs. clear examples)
- L2: Basic Prompt Structure (Task + Context + Format)
- L3: Adding Examples and Constraints (refining prompts manually)

**LAYER 2: AI Collaboration (L4-L6)** — Partner with AI using Three Roles
- L4: Iterative Refinement (Teacher/Student/Co-Worker roles)
- L5: Question-Driven Development (AI asks YOU questions first)
- L6: Validating AI Outputs (never trust blindly)

**LAYER 3: Intelligence Design (L7-L8)** — Create reusable patterns
- L7: Building Your Prompt Library (recognize recurring tasks)
- L8: Capstone - Your First Prompt Toolkit (5-7 templates with usage guide)

### Separation from Chapter 11 (Context Engineering)

**Chapter 10 Scope** (WHAT you SAY):
- Clear prompt writing (Task + Context + Format)
- Iteration and refinement
- Question-driven collaboration
- Prompt templates for common tasks
- Validation of AI outputs

**Chapter 11 Scope** (WHAT AI KNOWS):
- Context windows and token limits
- Progressive loading strategies
- Memory files (CLAUDE.md, architecture.md)
- Session management
- Context compression

**Prohibited in Chapter 10**:
- ❌ Context window explanations
- ❌ Token counting
- ❌ Memory file architecture
- ❌ Progressive loading techniques

---

## III. Learning Objectives (Measurable)

### Foundation Level (L1-L3: Manual)

**LO-001**: Student can distinguish vague from specific prompts
- **Assessment**: Given 5 prompt pairs, identify which produces better results and explain why
- **Example**: "help with git" (vague) vs. "explain git status output showing modified files" (specific)

**LO-002**: Student can structure basic prompts using Task + Context + Format
- **Assessment**: Write 3 prompts (Bash, Git, Markdown) using three-element structure
- **Example**: "Explain [Task] the git rebase command, assuming [Context] I understand merge, in [Format] 3 bullet points"

**LO-003**: Student can enhance prompts with examples and constraints
- **Assessment**: Take basic prompt and add 2 constraints + 1 example
- **Example**: Add "use beginner language" + "similar to how you explained merge" + "max 100 words"

### Application Level (L4-L6: AI Collaboration)

**LO-004**: Student demonstrates Three Roles in prompt refinement session
- **AI as Teacher**: AI suggests improvements student didn't know
- **AI as Student**: Student teaches AI their specific context
- **AI as Co-Worker**: Iteration produces better result than either had initially
- **Assessment**: Screen recording or transcript showing all three roles

**LO-005**: Student applies Question-Driven Development
- **Assessment**: Prompt AI to ask 5+ clarifying questions before solving task
- **Example**: "Before explaining git workflows, ask me 5 questions about my experience and project needs"

**LO-006**: Student validates AI outputs using checklist
- **Assessment**: Review AI-generated Bash script, identify 2 issues, prompt for fixes
- **Checklist**: Read → Understand → Test → Question unfamiliar parts

### Integration Level (L7-L8: Intelligence Design)

**LO-007**: Student creates 3+ reusable prompt templates
- **Assessment**: Submit prompt library (markdown file) with templates for recurring tasks
- **Requirements**: Each template has placeholders and usage notes
- **Example**: Git commit message template, Bash debugging template, Markdown doc structure template

**LO-008**: Student documents WHEN to use each template
- **Assessment**: Create decision guide (not just template list)
- **Example**: "Use Git commit template when..." / "Use Bash debug template when..."

---

## IV. Research Foundation

### Official Prompting Guides (Verified Sources)

**Anthropic Claude Docs** (https://docs.claude.com/claude/docs/prompt-engineering):
- Progressive approach: Clear/direct → Examples → Thinking → Advanced
- "Organized from most broadly effective to specialized techniques"
- Application: Chapter 10 follows this beginner → advanced progression

**Google Gemini Docs** (https://ai.google.dev/gemini-api/docs/prompting-strategies):
- Structure: Instructions → Context → Examples → Prefixes
- "Few-shot prompts often used to regulate formatting"
- Application: Task + Context + Format mirrors this structure

**OpenAI GPT Guide** (https://cookbook.openai.com/examples/gpt-5/gpt-5_prompting_guide):
- Levels: Basic direction → Constraints → Iteration → Reflexive
- "Vague/contradictory instructions MORE damaging in advanced models"
- Application: Emphasize clarity from L1

**Zia's 8-Element Framework** (https://github.com/ziaukhan/colearning-agentic-ai-specs):
- Command, Context, Logic, Roleplay, Formatting, Constraints, Examples, Iterative Questions
- Application: Simplified to Task + Context + Format for A2 (expanded in later lessons)

### Jake Heller Framework (Referenced, Not Primary)

**Source**: Y Combinator talk, 2024 (https://www.youtube.com/watch?v=l0h3nAW13ao)
- Quote: "Spend weeks tweaking prompts to get from 60% → 97%+" [20:03]
- Application: Illustrates value of iteration (L4), but NOT taught as primary framework
- Rationale: Professional-level case study, too advanced for A2 students to apply

**Usage in Chapter 10**:
- ✅ Mention as "Expert Insight" sidebar in L4 (iteration lesson)
- ❌ NOT taught as framework to memorize or apply
- ❌ NOT used in assessment criteria

### Removed Unverified Claims
- ❌ "55% more productive" → No authoritative source
- ❌ "8-element AIDD framework" → Doesn't exist; confused with 9 Pillars or Zia's framework

---

## V. Constraints

### Constitutional Compliance

**Mandatory Frameworks**:
- **4-Layer Teaching Method**: L1-L3 (Manual) → L4-L6 (Collaboration) → L7-L8 (Intelligence)
- **Progressive Complexity**: A2 (5-7 concepts) → B1 (7-10 concepts)
- **Factual Accuracy**: All claims verified, sources cited
- **Anti-Convergence**: Chapter 9 = direct teaching → Chapter 10 = experiential collaboration
- **Minimal Content**: ONLY prompt engineering, NO context engineering

**Student-Facing Language Protocol**:
- ❌ NO meta-commentary: "Notice how AI teaches you patterns"
- ❌ NO scaffolding exposure: "This is Layer 2, so we use Three Roles"
- ❌ NO framework labels: "## Three Roles Framework"
- ✅ Students EXPERIENCE pedagogy, don't STUDY it

**Lesson Ending Protocol**:
- ✅ ONLY final section: "Try With AI"
- ❌ NO "What's Next", "Key Takeaways", "Summary", standalone "Safety Note"

### Technical Constraints

**No Python Code**:
- Students have NOT learned Python yet (Part 4 starts Chapter 12)
- Practice vehicles: **Bash, Git, Markdown ONLY**
- When showing "code" examples, use Bash scripts with clear labels

**Platform Coverage**:
- Claude Code (Anthropic)
- Gemini CLI (Google)
- Both use natural language prompts (minimal syntax differences)
- Detailed tool comparison → Chapter 11

**Factual Accuracy**:
- ALL claims must have authoritative sources
- Frameworks cited from official docs (Anthropic, Google, OpenAI)
- NO hallucinated statistics or frameworks
- If claim cannot be verified → REMOVE

### Pedagogical Constraints

**Teaching Modality** (Anti-Convergence):
- Chapter 9: Direct teaching (lecture-style markdown tutorial)
- Chapter 10: **Experiential collaboration** (hands-on practice with AI)
- Show outcomes FIRST, explain patterns SECOND (specification-first applied to pedagogy, not content)

**Three Roles Framework** (L4-L6):
- MUST demonstrate all three roles: Teacher, Student, Co-Worker
- MUST show bidirectional learning (not passive AI tool use)
- MUST include convergence loop (iteration improves understanding)
- **Experienced, not labeled** (students see it in action, don't study framework)

**Layer Progression** (Non-Negotiable):
- L1-L3: Manual (ZERO AI tool usage, build mental models)
- L4-L6: AI Collaboration (hands-on practice with AI partners)
- L7-L8: Intelligence Design (create reusable prompt patterns)
- NO jumping to templates without foundation

---

## VI. Lesson Breakdown (8 Lessons)

### L1: Why Prompting Matters (A2, 30min, 3 concepts)

**Core Concepts**:
1. Vague prompts → Poor AI responses (comparison exercise)
2. Specific prompts → Better AI responses (before/after)
3. AI as collaborator, not magic oracle (mental model)

**Structure**:
- Opening: Real scenario (asking for "help" vs. asking specific question)
- Section 1: Compare 5 prompt pairs (vague vs. specific)
- Section 2: Why clarity matters (AI needs context)
- Practice: Improve 3 vague prompts WITHOUT AI (manual exercise)
- Try With AI: Test improved prompts, compare results

**Practice Vehicle**: Bash command explanations
- Vague: "help with ls"
- Specific: "explain ls -la output showing file permissions and sizes"

**Assessment**: Can student identify why specific prompts work better?

---

### L2: Basic Prompt Structure (A2, 35min, 4 concepts)

**Core Concepts**:
1. Task (WHAT you want): Verb-first (explain, create, debug, optimize)
2. Context (Background AI needs): Project type, experience level
3. Format (HOW you want answer): Code block, bullet points, table
4. Three-element pattern: Task + Context + Format

**Structure**:
- Opening: Why structure matters (AI understands patterns)
- Section 1: Task element (action verbs)
- Section 2: Context element (background information)
- Section 3: Format element (output structure)
- Practice: Write 3 prompts using structure (Bash, Git, Markdown)
- Try With AI: Test prompts, evaluate responses

**Practice Vehicle**: Git explanations
- Task: "Explain git rebase"
- Context: "assuming I understand merge"
- Format: "in 3 bullet points with examples"

**Assessment**: Can student structure prompts using three elements?

---

### L3: Adding Examples and Constraints (A2, 40min, 5 concepts)

**Core Concepts**:
1. Examples show desired output style
2. Constraints limit scope (length, complexity, language)
3. Multiple examples improve consistency
4. Constraints prevent unwanted behavior
5. Iterative refinement (manual practice)

**Structure**:
- Opening: Why examples and constraints matter
- Section 1: Providing examples (show don't tell)
- Section 2: Adding constraints (length, style, level)
- Section 3: Combining both (examples + constraints)
- Practice: Enhance L2 prompts with examples/constraints
- Try With AI: Compare basic vs. enhanced prompts

**Practice Vehicle**: Markdown documentation
- Basic: "Create markdown doc for project README"
- Enhanced: "Create markdown README similar to [example], max 200 words, beginner-friendly language, include installation and usage sections"

**Assessment**: Can student enhance prompts with 2 constraints + 1 example?

---

### L4: Iterative Refinement (A2→B1, 40min, 5 concepts)

**Core Concepts**:
1. First prompt rarely perfect (iteration mindset)
2. AI as Teacher (suggests improvements you didn't know)
3. AI as Student (you teach AI your context)
4. AI as Co-Worker (iteration produces better result)
5. Convergence loop (human + AI = better than either alone)

**Structure**:
- Opening: Why iteration matters (Jake Heller 60%→97% sidebar)
- Section 1: Initial prompt → Evaluate response → Identify gaps
- Section 2: Refine prompt → Better response → Convergence
- Section 3: Three Roles in action (experienced, not labeled)
- Practice: 3-iteration session (start → refine → refine → final)
- Try With AI: Full iteration loop on Bash script debugging

**Practice Vehicle**: Bash script debugging
- Iteration 1: "Debug this script" → Generic response
- Iteration 2: "Debug this script; it fails with 'permission denied' on line 5" → Specific fix
- Iteration 3: "Explain WHY this permission issue happens" → Understanding

**Assessment**: Screen recording/transcript showing all Three Roles

---

### L5: Question-Driven Development (B1, 45min, 6 concepts)

**Core Concepts**:
1. AI can ask YOU questions before answering
2. Questions reveal assumptions and gaps
3. Your answers provide context AI needs
4. Question-driven produces tailored solutions
5. Comparison: direct prompt vs. question-driven
6. When to use question-driven (complex tasks)

**Structure**:
- Opening: AI as technical consultant (asks before solving)
- Section 1: Prompting AI to ask questions
- Section 2: Answering questions thoughtfully
- Section 3: Compare direct vs. question-driven results
- Practice: Complex task (git workflow design) using QDD
- Try With AI: "Before explaining git workflows, ask me 5 questions about my experience and project needs"

**Practice Vehicle**: Git workflow design
- Direct: "Explain git workflows" → Generic answer
- QDD: AI asks about team size, project type, release cadence → Tailored answer

**Assessment**: Can student initiate QDD for complex task?

---

### L6: Validating AI Outputs (B1, 40min, 6 concepts)

**Core Concepts**:
1. Never trust AI blindly (validation mindset)
2. Validation checklist: Read → Understand → Test → Question
3. Red flags: Overly complex, missing error handling, unexplained magic
4. Ask AI to explain unfamiliar parts
5. Test outputs in safe environment
6. Iterate when validation fails

**Structure**:
- Opening: Why validation matters (AI makes mistakes)
- Section 1: Validation checklist (systematic approach)
- Section 2: Red flag patterns (security, complexity, errors)
- Section 3: Prompting for explanations
- Practice: Review AI-generated Bash script, identify issues, prompt fixes
- Try With AI: Validate and improve AI-generated solution

**Practice Vehicle**: Bash script validation
- AI generates script with hardcoded path → Student identifies red flag
- Student prompts: "Why is this path hardcoded? How can I make it configurable?"
- AI explains and provides improved version

**Assessment**: Can student identify 2 issues and prompt for fixes?

---

### L7: Building Your Prompt Library (B1, 45min, 7 concepts)

**Core Concepts**:
1. Recognize recurring tasks (pattern recognition)
2. Extract reusable structure (template creation)
3. Placeholders for variables (generalization)
4. Usage notes for context (decision guide)
5. Template categories (explain, debug, create, optimize)
6. Version control for templates (iterate and improve)
7. Test templates on new scenarios (validation)

**Structure**:
- Opening: Why build library (efficiency + consistency)
- Section 1: Identify recurring tasks (Bash, Git, Markdown)
- Section 2: Extract template structure (placeholders)
- Section 3: Document usage criteria (when to use)
- Practice: Create 3 templates (Git commit, Bash debug, Markdown doc)
- Try With AI: Test templates on new scenarios, refine

**Practice Vehicle**: Template creation
- Git commit template: "Write commit message for [FEATURE] that [ACTION], following [STYLE]"
- Bash debug template: "Debug Bash script [FILENAME] failing with [ERROR] at [LINE]"
- Markdown doc template: "Create markdown doc for [PROJECT] including [SECTIONS]"

**Assessment**: Submit 3 templates with placeholders + usage notes

---

### L8: Capstone - Your First Prompt Toolkit (B1, 60min, integration)

**Core Concepts**:
- Integration of all skills (L1-L7)
- Build comprehensive prompt library (5-7 templates)
- Document WHEN to use each (decision guide)
- Validate templates through testing
- Peer review validation (can others use your toolkit?)

**Structure**:
- Opening: Build personal prompt toolkit (portfolio piece)
- Section 1: Audit recurring tasks (what do YOU do often?)
- Section 2: Create 5-7 templates covering tasks
- Section 3: Decision guide (task → template mapping)
- Section 4: Test each template on real scenario
- Practice: Build complete toolkit (markdown file)
- Try With AI: Validate toolkit with AI feedback, peer review

**Deliverable**: Markdown file containing:
```markdown
# My Prompt Toolkit

## Template 1: [Name]
**When to use**: [Criteria]
**Template**: [Prompt with placeholders]
**Example**: [Filled-in version]

## Template 2: [Name]
...

## Decision Guide
- For [task type] → Use [template name]
- For [task type] → Use [template name]
```

**Assessment**:
- Toolkit has 5-7 templates
- Each has usage criteria
- Decision guide included
- Peer can use without clarification

---

## VII. Non-Goals: What We're NOT Building

### Explicitly Excluded

**Context Engineering** (Chapter 11):
- Context windows, token limits
- Progressive loading
- Memory files (CLAUDE.md, architecture.md)
- Session management

**Python Programming** (Part 4):
- Python syntax or examples
- Python-specific prompting
- Python debugging with AI

**Advanced AI Concepts** (Beyond Part 3):
- Fine-tuning, RAG
- Agentic workflows
- LLM internals

**Professional Frameworks** (Beyond A2-B1):
- Jake Heller framework as primary teaching tool
- Product management terminology
- Specification writing (Part 5)
- "WHAT vs HOW" abstraction (requires programming context)

### Why These Are Excluded

**Context Engineering**: Chapter 11 teaches this; clean separation improves learning

**Python**: Students haven't learned programming yet; premature examples violate progressive complexity

**Professional Frameworks**: A2-B1 students need practical collaboration skills, not abstraction layers

---

## VIII. Acceptance Tests: How We Know It's Done

### Specification Completeness
- [ ] All 8 learning objectives (LO-001 through LO-008) have corresponding lesson content
- [ ] All claims cite authoritative sources (Anthropic, Google, OpenAI, Zia)
- [ ] Zero overlap with Chapter 11 content (no context engineering)
- [ ] Zero Python code examples (Bash/Git/Markdown only)

### Constitutional Compliance
- [ ] L1-L3 have ZERO AI tool usage (manual foundation)
- [ ] L4-L6 demonstrate Three Roles (experienced, not labeled)
- [ ] L7-L8 create reusable templates (intelligence design)
- [ ] Teaching modality varies from Chapter 9 (experiential vs. direct)
- [ ] All lessons end with ONLY "Try With AI" section
- [ ] Zero meta-commentary in student-facing text

### Pedagogical Quality
- [ ] Concept density respects tier limits (A2: 5-7, B1: 7-10)
- [ ] 8-lesson structure derived from concept analysis
- [ ] All practice uses appropriate vehicles (Bash, Git, Markdown)
- [ ] Three Roles experienced naturally (not studied as framework)
- [ ] Progressive: L1 (compare prompts) → L8 (build toolkit)

### Factual Accuracy
- [ ] Official docs cited (Anthropic, Google, OpenAI)
- [ ] Jake Heller referenced as sidebar (not primary framework)
- [ ] Zero unverified statistics
- [ ] Zero hallucinated frameworks

### Validation Rigor
- [ ] Technical review: All prompts tested with Claude Code/Gemini CLI
- [ ] Pedagogical review: Layer progression validated
- [ ] Factual review: All sources verified
- [ ] Constitutional review: All principles applied correctly

---

## IX. Success Criteria Summary

**This specification succeeds when**:

1. ✅ Students can communicate clearly with AI (Task + Context + Format)
2. ✅ Students demonstrate Three Roles in iteration (Teacher/Student/Co-Worker)
3. ✅ Students apply Question-Driven Development for complex tasks
4. ✅ Students validate AI outputs systematically (never trust blindly)
5. ✅ Students create reusable prompt templates (pattern recognition)
6. ✅ Students build personal prompt toolkit (5-7 templates with decision guide)
7. ✅ Zero overlap with context engineering (Chapter 11)
8. ✅ Zero Python examples (appropriate for Part 3, A2-B1)

**This specification fails when**:

1. ❌ Teaching "specification writing" instead of "clear communication"
2. ❌ Using professional frameworks (Jake Heller, PM terminology) as primary pedagogy
3. ❌ Context engineering concepts appear (violates Chapter 11 separation)
4. ❌ Python code examples used (violates Part 3 constraints)
5. ❌ Meta-commentary exposes scaffolding (violates student-facing protocol)
6. ❌ Three Roles taught as framework (should be experienced naturally)

---

## X. Next Steps

**After Specification Approval**:

1. **Update plan.md**: 8-lesson breakdown with teaching modality per lesson
2. **Update tasks.md**: Implementation tasks (one per lesson)
3. **Implementation**: Create 8 lesson files using collaboration-focused approach
4. **Validation**: Test prompts with Claude Code/Gemini CLI, verify sources

---

**Specification Version**: 2.0 (Collaboration-Focused)
**Ready for**: User approval and implementation
