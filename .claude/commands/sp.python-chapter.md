---
description: Intelligence-driven workflow for Python chapters (12-29). Reads constitution + chapter-index to derive audience/complexity/prerequisites automatically. Asks only targeted questions when genuinely ambiguous. Chains /sp.specify → /sp.plan → /sp.tasks → /sp.implement with validation gates.
---

# /sp.python-chapter: Intelligence-Driven Python Chapter Workflow

**Purpose**: Design a complete Python chapter (12-29) using **vertical intelligence** (constitution + chapter-index + skills) to automatically derive context. No hardcoded questions - the command reads authoritative sources and asks only what's genuinely ambiguous. Chains full workflow (Spec → Plan → Tasks → Implement → Validate) with approval gates.

**Intelligence Sources**:
- Constitution: Target audience, philosophy, learning patterns
- Chapter Index: Exact title (THE ANCHOR), part, prerequisites
- Skills Library: Available domain skills for this chapter
- Context Materials: Existing pedagogical patterns (if available)

**Adaptive Questions**: 0-3 targeted questions based on what intelligence can't derive.

## User Input

```text
$ARGUMENTS
```

## VERTICAL INTELLIGENCE: AIDD-Driven Python Teaching

Before orchestration begins, understand what makes Python chapters effective in the AI-native era:

### Core Principle: Specification-First, Validation-First, AI Partnership

Students don't memorize Python syntax. Instead:

1. **Understand the concept** (plain language explanation)
2. **See minimal code** (what it does in action)
3. **Ask their AI** (explore through dialogue with Claude Code/Gemini CLI)
4. **Extract insight** (why this matters for thinking, not just coding)

### AI-Native Learning for Part 4 Students

**Traditional Programming Teaching**:
- "Memorize Python syntax"
- "Here are all 47 string methods"
- Syntax-first (memorize, then apply)

**AI-Native Learning Pattern** (Part 4: Chapters 12-29):
- **Describe Intent**: Use type hints and clear code to communicate what data means
- **Explore with AI**: Ask AI questions to understand concepts (not memorize docs)
- **Validate Together**: Use isinstance(), type(), and tests to check understanding
- **Learn from Errors**: When errors occur, ask AI "why?" and learn the pattern

**Note on AIDD**: Students in Chapters 1-11 learned AIDD principles. Part 4 applies these principles to learning Python, using the beginner-friendly "AI-Native Learning" framing. Students don't write formal specifications yet (that's Part 5+), but they DO describe intent through type hints and clear code structure.

### Teaching Pattern (Every Concept)

```markdown
## 1. [Concept Name] — [Why it matters]

**What it is:**
Plain-language explanation (2-3 sentences).

### 💻 Code Idea

\`\`\`python
# Minimal code showing the concept
# Focus on WHAT it does
\`\`\`

### 🤖 Think With Your AI

> "What does this do?"
>
> "What changes if we...?"
>
> "How would you use this to...?"

### 🧠 The Reasoning Pattern

[Why this concept matters for thinking, not just coding]
```

**Example:**

```markdown
## 1. Variables — Storing Data

**What it is:**
A variable names a value so your program can remember it.

### 💻 Code Idea

\`\`\`python
name = "Alex"
score = 95
\`\`\`

### 🤖 Think With Your AI

> "Why do we need variables instead of just using 95?"
>
> "What breaks if we forget to name a value?"
>
> "How do AI agents use variables to track context?"

### 🧠 The Reasoning Pattern

Programs need memory. Variables let you say "remember this as X"—
exactly how reasoning chains in AI maintain state.
```

---

## Python Standards (Chapters 12-29)

**Version:** 3.14+ (always use latest stable release from https://www.python.org/downloads/)
**Syntax:** f-strings only, match/case (17+), modern types (`list[int]`, `X | None`)
**Type hints:** Core (Ch 13) → Gradual Application (14-26) → Mandatory (27+)
**Note on Type Hints:** Modern Python treats type hints as essential for clarity and specification-first thinking, not optional features. Integrate from Chapter 13 onwards.
**MCP Documentation Source**: Python.org official docs via context7 MCP server (loads current, authoritative reference)

**Security (non-negotiable):**
- ❌ No `eval()`, `shell=True`, hardcoded secrets
- ✅ Environment variables, input validation, modern patterns

---

## CRITICAL DESIGN RULES

### Rule 1: USER INTENT IS AUTHORITY

**Never override user input:**
- User says "beginner" → Make A1-A2 (NOT A2-B1)
- User says "just variables" → Only variables (NOT + functions + loops)
- User says "absolute beginners" → 5 concepts max, simple framing

**Always ask, always honor. Do NOT assume.**

---

### Rule 2: NO FORWARD REFERENCES + PART 4 APPROPRIATE LANGUAGE

**Never mention untaught concepts:**
- ❌ NO Chapter 30+ references
- ❌ NO "Specification-Driven Development" (not yet taught - that's Part 5+)
- ❌ NO "write a specification" (use "describe your intent" instead)
- ❌ NO professional SDD terminology for Part 4 students

**DO reference AI-Native Learning (appropriate for Part 4):**
- ✅ "Describe what your code should do using type hints..."
- ✅ "Ask your AI to explain this concept..."
- ✅ "Validate your understanding by testing the code..."
- ✅ "Learn from errors by asking AI 'why did this fail?'..."

**DO reference AIDD principles from Chapters 1-11 (context only):**
- ✅ "This applies the AIDD thinking you learned in Part 1..."
- ✅ "Remember the validation-first approach from Chapter 4..."
- ✅ "You're using AI as co-reasoning partner, not coding assistant..."

**Critical Distinction**:
- Part 4 students use **AI-Native Learning** (beginner-friendly: describe intent → explore → validate → learn from errors)
- Part 5+ students learn **Specification-Driven Development** (professional: write formal specs → generate → test → iterate)
- Type hints are "describing intent" NOT "writing specifications" in Part 4

---

### Rule 3: RUTHLESS CONTEXT FILTERING

**When extracting from context materials:**

**Chapter 13 "Introduction to Python":**
- ✅ "What is Python?" → USE (intro concept)
- ✅ "Your first program" → USE (intro outcome)
- ❌ "Functions" → SKIP (Ch 20 topic)
- ❌ "Classes" → SKIP (Ch 24+ topic)
- ❌ "Async/await" → SKIP (Ch 28 topic)

**Chapter 17 "Control Flow and Loops":**
- ✅ "if/elif/else statements" → USE (chapter focus)
- ✅ "for loops" → USE (chapter focus)
- ❌ "Functions" → SKIP (Ch 20 topic)
- ❌ "List comprehensions" → SKIP (advanced)
- ❌ "Exception handling" → SKIP (Ch 21 topic)

**Decision Rule:**
- IF context concept fits THIS chapter's title → EXTRACT
- IF context concept belongs to Ch N+1 or later → SKIP
- IF context concept is advanced variation → SKIP
- IF context concept requires future prerequisites → SKIP

---

### Rule 4: MINIMAL SCOPE

**Depth > breadth.**

- Beginner (Ch 12-16): 5 concepts max, 3-4 lessons
- Intermediate (Ch 17-23): 7 concepts max, 4-5 lessons
- Advanced (Ch 24-29): 10 concepts max, 5-6 lessons

---

### Rule 5: MINIMAL FILES

**Never create:**
- ❌ index.md, _templates/, _assets/, _code-examples/, lesson-template.md, capstone-rubric.md

---

### Rule 6: TROUBLESHOOTING IS AI PARTNERSHIP

**Real-world context:** In an AI-native world, students will encounter errors (installation, syntax, environment issues). Rather than detailed troubleshooting in every chapter, teach students to ASK their AI assistant.

**Application in chapters:**
- **Installation/Setup chapters**: Include prompt like: `"I tried to install Python but got this error: [error]. What does this mean and how do I fix it?"`
- **Execution chapters**: Include prompt like: `"My program runs but gives this output. Is this correct? Why?"`
- **Advanced chapters**: Include prompt like: `"I'm getting a TypeError. Walk me through what went wrong."`

**Why this works:**
- ✅ Teaches resilience: Errors are information to be understood, not obstacles
- ✅ Builds partnership: AI becomes problem-solving collaborator, not just code generator
- ✅ Scales with complexity: Works for simple errors (Python not found) to complex errors (type mismatches)
- ✅ Honors reality: Professional developers ask AI for error help constantly

**Example (from Chapter 13, Lesson 2):**
```markdown
### Prompt 2: Troubleshoot Installation Errors
\`\`\`
I tried to install Python but got this error: [describe your error].
What does this mean and how do I fix it?
\`\`\`

**Expected outcome**: AI explains the error and provides step-by-step fixing instructions.
```

This single prompt replaces 10 pages of platform-specific troubleshooting guides that become outdated.

---

### Rule 7: GRADUATED TEACHING PATTERN (Constitution Principle 13)

**Apply the three-tier teaching approach from the constitution:**

**Tier 1 - Foundational Concepts** (Book Teaches Directly):
- Stable, core concepts explained directly in book
- Direct explanation with analogies and examples
- Examples: What are variables? What is a loop? What are type hints?
- NO "Ask your AI: What is X?" for foundations
- Book provides clear, authoritative explanation first

**Tier 2 - Complex Syntax** (AI Companion Handles):
- Complex syntax patterns AI handles (student directs, AI executes)
- Student specifies WHAT they want, AI handles HOW
- Examples: Decorators, context managers, complex regex, async/await patterns
- "Tell your AI: Create X with these requirements..."
- Student learns strategy and intent, not memorization of syntax

**Tier 3 - Scaling Operations** (AI Orchestration):
- Operations involving 10+ items or multi-file workflows
- Student orchestrates strategy, AI manages tactical execution
- Examples: Setting up 10 test environments, batch file conversions, project scaffolding
- "Tell your AI: Set up 10 X with Y configuration..."
- Student learns orchestration and supervision skills

**Application to Part 4 (Chapters 12-29)**:
- Primarily Tier 1 (foundations) and Tier 2 (applied syntax)
- Tier 3 introduced gradually in advanced chapters (24-29)
- Balance: Book teaches concepts, AI handles complexity, student directs strategy

---

### Rule 8: STANDARDIZED "TRY WITH AI" FORMAT (End-of-Lesson Closure)

**Every lesson MUST end with "Try With AI" section ONLY** following this exact structure (verified in Chapter 1 and Chapter 13):

```markdown
## Try With AI

Use your AI companion (Claude Code or Gemini CLI). [Brief context about what you're exploring].

### Prompt 1: [Descriptive Title - Recall/Understand]
\`\`\`
[Clear, concrete prompt asking about the concept]
\`\`\`

**Expected outcome**: [What student should understand after AI response]

### Prompt 2: [Descriptive Title - Apply]
\`\`\`
[Clear, concrete prompt asking about application or edge case]
\`\`\`

**Expected outcome**: [What student learns from this]

### Prompt 3: [Descriptive Title - Analyze/Evaluate]
\`\`\`
[Prompt encouraging deeper understanding or connection to real-world use]
\`\`\`

**Expected outcome**: [Connection to AIDD or professional practice]

### Prompt 4: [Descriptive Title - Synthesis/Create]
\`\`\`
[Synthesis prompt pulling together concepts from lesson]
\`\`\`

**Expected outcome**: [Integration of understanding + forward-looking insight]
```

**Critical requirements:**
- ✅ Exactly 4 prompts per lesson (progressive Bloom's levels: Remember → Understand → Apply → Analyze/Synthesize)
- ✅ Prompts are CONCRETE and SPECIFIC (not "ask AI about X")
- ✅ Each prompt has explicit "Expected outcome" describing what student learns
- ✅ Prompts should include rubric-style validation ("Does this answer your spec?")
- ✅ No "Key Takeaways" or "Summary" sections after "Try With AI"
- ✅ "Try With AI" is the final substantive section (closure point)

**CRITICAL LESSON CLOSURE PATTERN** (Constitutional Mandate):

Lessons MUST end with "Try With AI" section ONLY. Prompt 4 provides cognitive closure.

**NEVER ADD after "Try With AI":**
- ❌ "Key Takeaways" or "Summary"
- ❌ "What's Next"
- ❌ "Completion Checklist" (even for capstone lessons)
- ❌ "Chapter Summary"
- ❌ Any other closure content

**WHY**: Try With AI Prompt 4 already provides reflection and synthesis. Additional sections create cognitive overload and violate Constitutional Rule 13. This was identified as a critical violation in Chapter 14 technical review.

**Why this matters:**
- Consistency across entire book (students know the format)
- Progressive prompts teach exploration, not memorization
- "Expected outcome" sets clear learning targets
- Validates understanding without artificial quizzes
- Prompt 4 synthesis provides natural closure

---

### Rule 9: AI-NATIVE COLEARNING PEDAGOGY (Throughout Lessons)

**CRITICAL**: Apply `ai-collaborate-teaching` skill throughout ALL lessons, not just at the end.

**CoLearning Structural Elements** (must appear throughout lesson content, NOT just "Try With AI" section):

#### 💬 AI Colearning Prompt (Claude Code or Gemini CLI)
- **When**: After introducing foundational concepts
- **Purpose**: Encourage deeper conceptual understanding with AI as co-reasoning partner
- **Format**:
```markdown
#### 💬 AI Colearning Prompt
> "Explain how [concept] works under the hood. Why did Python choose this design?"
```
- **Example**: "Explain how `for` loops work under the hood with iterators. Why does Python need both `for` and `while`?"

#### 🎓 Expert Insight: "From Syntax to Semantics"
- **When**: After code examples, before moving to next concept
- **Purpose**: Reframe learning goals (understanding > memorization); provide strategic depth
- **Key Mantra**: "Syntax is cheap — semantics is gold"
- **Format**:
```markdown
#### 🎓 Expert Insight
> In AI-native development, you don't memorize operator precedence—you understand when arithmetic matters and ask AI when confused. Syntax is cheap; understanding is gold.
```
- **Example**: "In AI-driven development, you don't memorize all 47 string methods—you understand what strings DO and ask AI: 'How do I format this string?'"

#### 🤝 Practice Exercise
- **When**: After students understand concept basics, ready for application
- **Purpose**: Hands-on collaborative practice with AI partnership; specification-driven thinking
- **Pattern**: Specification → AI Generation → Explanation → Understanding
- **Format**:
```markdown
#### 🤝 Practice Exercise

> **Ask your AI**: "Generate a function that calculates factorial using recursion. Then explain how recursion works step-by-step, including the call stack."

**Expected Outcome**: You'll understand recursion conceptually (not just syntax), see how AI generates code from specifications, and learn to validate AI output.
```
- **Example**: "Ask your AI: Generate a `for` loop that prints a multiplication table for 7. Then explain how `range()` works and why we use it instead of manual counting."

**Placement Guidelines by Proficiency Level**:

- **A1-A2 (Beginner)**:
  - 2-3 CoLearning elements per lesson
  - Emphasis on 💬 Prompts and 🎓 Insights (build confidence, reduce memorization anxiety)
  - Light on 🤝 Exercises (students still building foundations)

- **A2-B1 (Intermediate)**:
  - 3-4 CoLearning elements per lesson
  - Balance 💬 Prompts, 🎓 Insights, 🤝 Exercises

- **B1-B2 (Advanced)**:
  - 4-6 CoLearning elements per lesson
  - Emphasis on 🤝 Exercises (application and synthesis)
  - More 🎓 Insights (architectural decisions, tradeoffs)

**Tone Requirements for ALL Lessons**:
- ✅ Conversational (you, your, we)
- ✅ Exploration-focused (discover, explore, try)
- ✅ AI partnership emphasized (co-teacher, co-reasoning partner, pair-teacher)
- ❌ NOT documentation style
- ❌ NOT reference manual tone
- ❌ NOT traditional tutorial "here's how you do X" without AI collaboration context

**Critical Distinctions**:
- **CoLearning Elements** (throughout lesson): Conversational, exploration-focused, AI partnership throughout content
- **Try With AI Section** (end of lesson): Structured 4-prompt synthesis and reflection (closure point)

**Why This Matters**:
- Students learn WITH AI, not just USING AI
- AI positioned as intellectual partner, not autocomplete tool
- Builds critical thinking ("Why does this work?") not rote memorization
- Prepares for shipping era (professional AI-native development patterns)

**Validation**:
- validation-auditor MUST check for CoLearning elements throughout
- Missing 💬🎓🤝 = CRITICAL VIOLATION (regeneration required)
- Documentation tone (not conversational) = MAJOR VIOLATION (revision required)

---

## ORCHESTRATED WORKFLOW (What Actually Happens)

When you run `/sp.python-chapter [N]`:

### PHASE 0: Intelligent Context Gathering (Adaptive + MCP-Enhanced)

**Intelligence-Driven Discovery** (not hardcoded questions):

1. **Read authoritative sources**:
   - Constitution: `.specify/memory/constitution.md` (target audience, philosophy, principles)
   - Chapter Index: `specs/book/chapter-index.md` (chapter title, part, prerequisite chapters)
   - Skills Library: `.claude/skills/` (available domain skills, especially ai-collaborate-teaching)
   - Existing Context: `context/part-4-python/` or `context/13_chap12_to_29_specs/` (if available)
   - **MCP Documentation**: Python.org official docs via context7 MCP server (if available)

2. **Load Python Documentation via MCP** (WHEN AVAILABLE):
   - Use MCP tools to fetch Python.org official docs (v3.14+)
   - Load relevant sections for the chapter (tutorial, stdlib types, functions, chapter-specific libraries)
   - Graceful fallback to cached context if MCP unavailable
   - Acknowledge documentation source in outputs

3. **Derive chapter intelligence**:
   - **Audience**: From constitution (Aspiring/Professional/Founders with graduated complexity)
   - **Part**: From chapter-index.md (chapter N → Part X)
   - **Complexity Tier**: From chapter number range (12-16=beginner, 17-23=intermediate, 24-29=advanced)
   - **Prerequisite Knowledge**: All chapters 1 through N-1
   - **Chapter Title**: Exact title from chapter-index.md (THE ANCHOR)
   - **Learning Pattern**: AI-Native Learning (Part 4 appropriate, NOT formal SDD)

3. **Intelligently determine what to ask user** (context-adaptive):
   - IF context materials exist for this chapter → Ask: "Use existing context or start fresh?"
   - IF chapter title is ambiguous/broad → Ask: "What specific aspect should we emphasize?"
   - IF capstone vs foundational unclear → Ask: "Should students BUILD something or learn concepts?"
   - IF multiple teaching approaches possible → Ask: "Which pedagogical angle fits best?"

   **DO NOT ask**:
   - ❌ "Who is the audience?" (constitution already defines this)
   - ❌ "How many lessons?" (let intelligence determine based on scope)
   - ❌ "What CEFR level?" (derive from chapter number range automatically)

4. **Store derived intelligence** for next phases

**Apply Vertical Intelligence**: Constitution + Chapter Index + Skills → Adaptive questions (not hardcoded forms).

**CRITICAL**: Do NOT create git branch yet. Branch creation happens in Phase 1 AFTER spec.md is created, ensuring branch name matches spec directory name.

---

### PHASE 1: Specification (Automated + Quality Gate)

```
→ Invoke: /sp.specify [chapter-context]
  ├─ Pass: chapter number, title, derived intelligence, context materials
  ├─ Apply: AI-Native Learning principles, cognitive load limits, teaching patterns
  ├─ Create: specs/part-4-chapter-[N]/spec.md
  └─ Report: "Spec created."

→ Invoke: /sp.clarify (Quality Gate)
  ├─ Read: specs/part-4-chapter-[N]/spec.md
  ├─ Identify: Underspecified areas, ambiguities, missing details
  ├─ Ask: Up to 5 targeted clarification questions
  ├─ Update: spec.md with answers encoded
  └─ Report: "Spec clarified and updated."

→ Create Feature Branch (AFTER spec exists)
  ├─ Derive branch name from spec directory (e.g., specs/part-4-chapter-15/ → part-4-chapter-15)
  ├─ Check if already on correct branch:
  │   IF current branch == main → Create new branch matching spec directory
  │   IF current branch matches spec directory → Stay on it
  │   IF current branch != spec directory → Warn and ask user to switch
  ├─ Execute: git checkout -b [spec-directory-name] (only if on main)
  └─ Report: "✅ Branch created: [branch-name]" or "ℹ️  Already on branch: [branch-name]"

WAIT: User reviews spec.md
→ User confirms: "✅ Spec approved" or provides feedback
  ├─ If feedback: Update spec.md iteratively (may re-run /sp.clarify)
  └─ If approved: Continue to PHASE 2
```

**What /sp.specify receives:**
- Chapter title (anchor from chapter-index.md)
- User's audience answer (determines complexity tier: A1/A2/B1)
- User's scope answer (limits concepts to 5/7/10)
- User's outcome answer (real thing students will build)
- Context materials (extracted pedagogically)
- AI-Native Learning pattern (describe intent → explore → validate → learn from errors)
- Teaching pattern template (What it is → Code → Try → Why it matters)
- Cognitive load limits (max 5 for beginner, 7 for intermediate, 10 for advanced)

---

### PHASE 2: Planning (Automated + ADR Gate)

```
→ Invoke: /sp.plan [spec-context]
  ├─ Read: specs/part-4-chapter-[N]/spec.md (clarified)
  ├─ Apply: Lesson progression, CEFR proficiency levels, AI prompts, skills-proficiency-mapper
  ├─ Create: specs/part-4-chapter-[N]/plan.md
  └─ Report: "Plan created."

→ Invoke: /sp.adr (Architectural Decision Gate)
  ├─ Read: specs/part-4-chapter-[N]/plan.md
  ├─ Detect: Architecturally significant decisions (lesson structure, pedagogical approaches, tech choices)
  ├─ Suggest: "📋 Architectural decision detected: [X]. Document with /sp.adr [title]?"
  ├─ Wait: User consent to create ADR (never auto-create)
  ├─ Create: history/adr/[NNN]-[decision-title].md (if user approves)
  └─ Report: "ADR created and linked to plan." OR "ADR suggestion noted."

WAIT: User reviews plan.md (+ any ADRs)
→ User confirms: "✅ Plan approved" or provides feedback
  ├─ If feedback: Update plan.md iteratively
  └─ If approved: Continue to PHASE 3
```

**What /sp.plan receives:**
- Approved spec.md (learning objectives, concepts, success criteria)
- Chapter scope (what fits this chapter, what doesn't)
- AI-Native Learning pattern (Describe intent → Explore → Validate → Learn from errors)
- Proficiency expectations (CEFR A1/A2/B1 levels)
- Real outcome students will build
- Skills proficiency mapper for CEFR validation and cognitive load checks

---

### PHASE 3: Tasks (Automated + Analysis Gate)

```
→ Invoke: /sp.tasks [spec+plan-context]
  ├─ Read: specs/part-4-chapter-[N]/spec.md + plan.md
  ├─ Apply: Acceptance criteria, validation steps, implementation checklist
  ├─ Create: specs/part-4-chapter-[N]/tasks.md
  └─ Report: "Tasks created."

→ Invoke: /sp.analyze (Cross-Artifact Consistency Gate)
  ├─ Read: specs/part-4-chapter-[N]/spec.md + plan.md + tasks.md
  ├─ Validate: Cross-artifact consistency (spec ↔ plan ↔ tasks alignment)
  ├─ Check: Learning objectives → lessons → tasks traceability
  ├─ Detect: Missing tasks, orphaned objectives, scope drift, conflicts
  ├─ Report: Consistency issues (critical/major/minor) with recommendations
  └─ Output: analysis-report.md with findings

WAIT: User reviews tasks.md + analysis report
→ User confirms: "✅ Tasks approved" or provides feedback
  ├─ If critical issues: Must fix before proceeding
  ├─ If major issues: Should fix (user decision)
  ├─ If minor issues: Nice to fix (user decision)
  └─ If approved: Continue to PHASE 4
```

**What /sp.tasks receives:**
- Approved spec.md + plan.md (complete design)
- Learning objectives (what success looks like)
- Lessons (what needs to be implemented)
- Acceptance criteria (how to validate)

---

### PHASE 4: Implementation (Automated + Technical Review Gate)

```
→ Invoke: /sp.implement [chapter-slug]
  ├─ Read: specs/part-4-chapter-[N]/spec.md + plan.md + tasks.md (all approved)
  ├─ Strategy: Parallel team approach (Lessons 1-4 parallel, Lesson 5 sequential capstone)
  ├─ Invoke: content-implementer subagent (per lesson) WITH EXPLICIT COLEARNING INSTRUCTIONS
  ├─ Pass to content-implementer:
  │   CRITICAL INSTRUCTIONS FOR content-implementer:
  │
  │   Apply these domain skills IN THIS ORDER:
  │   1. ai-collaborate-teaching (CoLearning pedagogy THROUGHOUT lesson)
  │   2. learning-objectives (aligned with CEFR proficiency levels)
  │   3. concept-scaffolding (graduated complexity)
  │   4. code-example-generator (Python 3.14+, type hints)
  │   5. exercise-designer (deliberate practice)
  │
  │   CoLearning Structural Elements (MUST appear throughout lesson):
  │   - 💬 AI Colearning Prompt: After foundational concepts, encourage deeper exploration
  │   - 🎓 Expert Insight: Provide strategic depth; "syntax cheap, semantics gold" mantra
  │   - 🤝 Practice Exercise: Hands-on collaborative practice with AI partnership
  │
  │   Tone Requirements:
  │   - ✅ Conversational (you, your, we)
  │   - ✅ Exploration-focused (discover, explore, try)
  │   - ✅ AI partnership (co-teacher, pair-teacher)
  │   - ❌ NOT documentation style
  │   - ❌ NOT reference manual
  │
  │   Lesson Closure:
  │   - ✅ ONLY "Try With AI" section at end (4 prompts, Bloom's progression)
  │   - ❌ NO summaries, checklists, "what's next" after Try With AI
  │
  │   CRITICAL PEDAGOGICAL ORDERING RULES (MUST ENFORCE):
  │
  │   **Rule 1: NO FORWARD REFERENCES WITHIN CHAPTER**
  │   - ONLY use concepts/methods/functions taught in PREVIOUS lessons of this chapter
  │   - NEVER use concepts from CURRENT or FUTURE lessons as examples
  │   - Example VIOLATION: Using .upper() method in Lesson 1 when string methods are taught in Lesson 2
  │   - Example CORRECT: In Lesson 1, use only string creation, indexing, len(), +, * (concepts taught IN Lesson 1)
  │
  │   **Rule 2: INTRODUCE BEFORE USE**
  │   - Every method, function, or concept MUST be introduced BEFORE first use
  │   - Introduction means: explain what it is, what it does, why it matters
  │   - Example VIOLATION: Using len() without explaining it's a built-in function
  │   - Example CORRECT: "Python provides built-in functions like len() that work on many types. len() counts characters in a string."
  │
  │   **Rule 3: DISTINGUISH BUILT-INS FROM METHODS**
  │   - Built-in functions (len, type, isinstance): Explain they're "Python's built-in tools"
  │   - Methods (.upper, .split, .join): Explain they're "actions strings can do"
  │   - Always clarify: "len() is a built-in function, not a string method"
  │
  │   **Rule 4: CONCEPT PREREQUISITE VALIDATION**
  │   Before writing any code example, ask:
  │   - "Have all concepts in this example been taught in THIS lesson or PRIOR lessons?"
  │   - "Do students have the prerequisite knowledge to understand this?"
  │   - "Am I introducing anything new without explanation?"
  │
  │   **Rule 5: LESSON BOUNDARY ENFORCEMENT**
  │   - Lesson 1 concepts ONLY available in Lesson 1
  │   - Lesson 1 + Lesson 2 concepts available in Lesson 2
  │   - Lesson 1-3 concepts available in Lesson 3
  │   - Capstone: ALL chapter concepts available (but NO new concepts introduced)
  │
  │   [Full context: spec, plan, tasks, MCP docs, AI-Native Learning pattern, CEFR levels]
  ├─ Apply: AI-Native Learning pattern, CEFR levels, validation-first approach, CoLearning throughout
  ├─ Create: book-source/docs/04-Part-4-Python-Fundamentals/[N]-[chapter-name]/
  │   ├─ readme.md
  │   ├─ 01-[lesson-name].md (with 💬🎓🤝 throughout)
  │   ├─ 02-[lesson-name].md (with 💬🎓🤝 throughout)
  │   ├─ 03-[lesson-name].md (with 💬🎓🤝 throughout)
  │   ├─ 04-[lesson-name].md (with 💬🎓🤝 throughout)
  │   └─ 05-[capstone-name].md (if applicable, with 💬🎓🤝 throughout)
  └─ Report: "All lessons implemented with CoLearning pedagogy."

→ Invoke: validation-auditor (Quality Gate)
  ├─ Read: All lesson files
  ├─ Validate: AI-Native CoLearning compliance (💬🎓🤝 elements present throughout)
  ├─ Check: Conversational tone (not documentation style)
  ├─ Check: Lesson closure pattern (Try With AI ONLY, no summaries)
  ├─ Check: Part 4 language appropriateness, constitutional alignment
  ├─ Test: All code examples (Python 3.14+, modern type hints)
  ├─ **NEW: Check: Pedagogical Ordering Compliance (CRITICAL)**
  │   ├─ Scan each lesson for forward references:
  │   │   - Lesson 1: Only uses concepts taught IN Lesson 1
  │   │   - Lesson 2: Only uses Lesson 1 + Lesson 2 concepts
  │   │   - Lesson N: Only uses Lessons 1 through N concepts
  │   ├─ Verify all methods/functions introduced before use:
  │   │   - First use of any method MUST have explanation
  │   │   - Built-in functions (len, type, isinstance) explained as "Python's built-in tools"
  │   │   - String methods (.upper, .split) explained as "actions strings can do"
  │   ├─ Flag violations:
  │   │   - CRITICAL: Using .upper() in Lesson 1 when methods taught in Lesson 2
  │   │   - CRITICAL: Using len() without explaining it's a built-in function
  │   │   - CRITICAL: Any concept used before introduction
  │   └─ Report: List all forward references and missing introductions
  ├─ Report: Validation report with PASS/CONDITIONAL PASS/FAIL
  └─ Output: VALIDATION_REPORT_CHAPTER_[N].md

→ If CONDITIONAL PASS or FAIL:
  ├─ Apply fixes for critical issues (especially missing CoLearning elements)
  ├─ Re-run validation-auditor
  └─ Repeat until PASS

WAIT: User reviews lessons + validation report
→ User confirms: "✅ Implementation approved"
  └─ Proceed to PHASE 5 (finalization)
```

---

### PHASE 5: Finalization (Update Chapter Index)

```
→ Update: specs/book/chapter-index.md
  ├─ Find: Chapter N row in Part 4 table
  ├─ Update status: 📋 Planned → ✅ Implemented & Validated
  ├─ Update Implementation Status section at top:
  │   ├─ Increment count: "X chapters" → "X+1 chapters"
  │   └─ Add Chapter N status block with:
  │       - Number of lessons implemented
  │       - Technical review result (PASS + any critical issues fixed)
  │       - Key features (AI-Native Learning, type hints, complexity tier)
  │       - Date (YYYY-MM-DD format)
  └─ Report: "Chapter index updated"

→ Optional: Create commit and PR
  ├─ User may request: "/sp.git.commit_pr" for automated git workflow
  └─ Or: Manual commit with summary of chapter completion
```

**Chapter Index Update Pattern**:
```markdown
- ✅ **Implemented & Validated** (X chapters): Chapters 1-N, 30-33...
  - **Chapter N Status**: ✅ COMPLETE + VALIDATED (YYYY-MM-DD)
    - [lessons-count] lessons written with AI-Native Learning pattern
    - Technical review [PASS/CONDITIONAL PASS] ([critical-issues-count] critical issues fixed)
    - [key-features]: Type hints, "Try With AI" format, graduated complexity
```

---

## KEY PRINCIPLES (Always Applied)

### ✅ Take Context, Discuss, Make Chapters (The AI-Native Workflow)

**The "Shipping Era" Approach**:
1. **Take Context**: Load authoritative sources (constitution, chapter-index, MCP docs, existing materials)
2. **Discuss**: Engage with user to understand intent, clarify ambiguities, align on goals
3. **Make Chapters**: Generate production-ready content with built-in quality (CoLearning, validation, proficiency-mapping)

This workflow ensures:
- Context-aware generation (not generic templates)
- Human-AI collaboration (not autonomous generation)
- Quality built-in (not bolted on afterwards)
- Shipping-ready output (not drafts requiring major revision)

### ✅ AI-Native CoLearning Pedagogy First (Rule 9)
- Apply `ai-collaborate-teaching` skill THROUGHOUT lessons (not just end)
- CoLearning elements (💬🎓🤝) positioned strategically in every lesson
- Conversational tone (you, your, we) - NOT documentation style
- AI positioned as co-reasoning partner, not autocomplete tool
- 40/40/20 balance: Foundation 40%, AI-Assisted 40%, Verification 20%
- "Syntax is cheap — semantics is gold" mantra reinforced

### ✅ MCP-Enhanced Intelligence (When Available)
- Load official Python documentation via context7 MCP server
- Fallback to cached context if MCP unavailable
- Reference docs for technical accuracy throughout workflow
- Documentation sources explicitly acknowledged

### ✅ AI-Native Learning First (Part 4 Appropriate)
- Apply AI-Native Learning pattern: describe intent → explore → validate → learn from errors
- Reference AIDD principles from Chapters 1-11 for context (not formal methodology)
- Validation-first practice: "How will students test understanding?"
- AI partnership: "How will they use Claude Code/Gemini CLI as co-reasoning partners?"
- NO formal "specification writing" (that's Part 5+) - use "describe intent" framing

### ✅ No Forward References
- Zero mentions of Chapters 30+ (SDD taught later)
- No "Specification-Driven Development" terminology (use "AI-Native Learning")
- No concepts from future chapters
- Chapter title from `chapter-index.md` is the absolute anchor

### ✅ Honors User Intent
- User's audience choice = final decision (never override)
- User's scope answer = limits concepts (never expand)
- User's outcome answer = determines lessons (never modify)

### ✅ Ruthless Context Filtering
- Only extract context matching THIS chapter's title
- Skip concepts from future chapters (even if in materials)
- Skip advanced variations and tangential concepts

### ✅ Cognitive Load Limits
- Max 5 concepts for beginner (Ch 12-16)
- Max 7 concepts for intermediate (Ch 17-23)
- Max 10 concepts for advanced (Ch 24-29)

### ✅ Teaching Intelligence Preserved
- Every phase applies AI-Native CoLearning principles
- Every phase uses teaching patterns (Book → AI Companion → AI Orchestration)
- Every phase respects chapter boundaries
- Every phase validates against acceptance criteria
- Skills proficiency mapping applied in planning phase (CEFR levels, cognitive load)

---

## EXECUTION INSTRUCTIONS (For Claude Code)

**CRITICAL**: This is an EXECUTABLE orchestration prompt, not documentation. Claude Code must:
1. Follow this flow exactly, in this order
2. Automatically invoke downstream commands WITHOUT asking for approval first
3. Pass full context (AIDD principles, teaching patterns) to each command
4. Respect approval gates ONLY between phases (not before first invocation)

### THE ORCHESTRATED WORKFLOW (EXECUTABLE)

#### PHASE 0: Intelligent Context Discovery (Adaptive, NOT Hardcoded)

**1. Read Authoritative Sources** (Automatic, NO USER INTERACTION):
- Constitution (`.specify/memory/constitution.md`): audience, philosophy, principles
- Chapter index (`specs/book/chapter-index.md`): title, file name, part number
- Available skills (`.claude/skills/` directory)
- Existing context materials (`context/` directory, if any)

**2. Derive Chapter Intelligence** (Automatic computation):

```python
# From constitution (no need to ask user)
audience = "Aspiring/Professional/Founders (graduated complexity)"

# From chapter number (automatic tier assignment)
if 12 <= chapter_num <= 16:
    complexity_tier = "beginner"
    cefr_range = "A1-A2"
    max_concepts = 5
elif 17 <= chapter_num <= 23:
    complexity_tier = "intermediate"
    cefr_range = "A2-B1"
    max_concepts = 7
elif 24 <= chapter_num <= 29:
    complexity_tier = "advanced"
    cefr_range = "B1-B2"
    max_concepts = 10

# From chapter index (THE ANCHOR)
part_num = 4  # Chapters 12-29 are Part 4
prerequisites = f"Chapters 1-{chapter_num - 1}"
learning_pattern = "AI-Native Learning"  # Part 4 appropriate

# Store derived intelligence
chapter_intelligence = {
    "number": chapter_num,
    "title": chapter_title,  # FROM CHAPTER-INDEX.MD (authoritative)
    "part": part_num,
    "complexity_tier": complexity_tier,
    "cefr_range": cefr_range,
    "max_concepts_per_lesson": max_concepts,
    "prerequisites": prerequisites,
    "audience": audience,
    "learning_pattern": learning_pattern,
    "available_skills": skills
}
```

**3. Intelligently Determine What to Ask** (Context-adaptive):
- Only ask if genuinely ambiguous or requires human judgment
- Example triggers: existing context found, broad chapter title, unclear capstone vs conceptual
- Ask 0-3 targeted questions max (NOT hardcoded)
- Store user preferences in chapter intelligence

**Key Principle**: Intelligence derives from constitution + chapter-index + skills library. Only ask user when GENUINELY ambiguous or requires human creative judgment.

**CRITICAL**: Do NOT create git branch in Phase 0. Branch creation happens in Phase 1 AFTER spec.md is created (see Phase 1 workflow).

---

#### PHASE 1: Specification (Automated + Intelligent)

**THIS PHASE INVOKES `/sp.specify` AUTOMATICALLY WITH FULL CONTEXT**

1. **Prepare context** (Ruthless filtering applied):
   - Gather user's 4 answers from PHASE 0
   - Extract materials from context directories (if available):
     - `context/13_chap12_to_29_specs/` (legacy structure)
     - `context/part-4-python/` (preferred structure)
     - Skip if no context available (spec from scratch is valid)
   - Apply ruthless filtering: Skip future chapters, skip advanced variations, skip tangential concepts
   - Embed AI-Native Learning principles in the context
   - Embed teaching patterns in the context (Book → AI Companion → AI Orchestration)
   - Embed cognitive load limits (5 for beginner, 7 for intermediate, 10 for advanced)

2. **Invoke /sp.specify with full context**:
   ```
   /sp.specify [chapter-slug]

   Write Chapter [N]: [Title] in Part [P]

   [Full AIDD context, user answers, teaching patterns, cognitive load rules, ruthlessly filtered context materials]
   ```

3. **Wait for /sp.specify completion**:
   - ✅ `specs/part-[P]-chapter-[N]/spec.md` is created
   - ✅ AIDD principles applied
   - ✅ Teaching patterns specified
   - ✅ Learning objectives aligned with evals

4. **Output approval checkpoint**:
   ```
   ✅ PHASE 1 COMPLETE: Specification Created

   📋 specs/part-[P]-chapter-[N]/spec.md

   Please review and confirm:
   - ✅ "Spec approved" to proceed to planning
   - 📝 Feedback to revise specification
   - ❓ Questions for clarification
   ```

5. **Wait for user approval**: Block here until user confirms "✅ Spec approved" OR provides feedback

---

#### PHASE 2: Planning (Automated + Intelligent) - Triggered After Spec Approval

**THIS PHASE INVOKES `/sp.plan` AUTOMATICALLY WITH FULL CONTEXT**

1. **Prepare context** (Read approved spec, add intelligence):
   - Read: `specs/part-[P]-chapter-[N]/spec.md` (the approved specification)
   - Extract: Learning objectives, key concepts, success criteria
   - Add: CEFR proficiency levels (A1/A2/B1 based on audience)
   - Add: Skills proficiency mapping (identify skills, assign CEFR levels, validate progression)
   - Add: Cognitive load validation (max concepts per lesson based on proficiency)
   - Add: Bloom's taxonomy alignment (cognitive complexity matching proficiency level)
   - Add: Lesson progression rules (foundational → applied → integration)
   - Add: AI prompts for each lesson (validation-first approach)
   - Add: Teaching pattern structure for every lesson (Book → AI Companion → AI Orchestration)

2. **Invoke /sp.plan with full context**:
   ```
   /sp.plan [chapter-slug]

   [Full context from spec, CEFR levels, lesson structure, AIDD teaching patterns]
   ```

3. **Wait for /sp.plan completion**:
   - ✅ `specs/part-[P]-chapter-[N]/plan.md` is created
   - ✅ Lessons broken down lesson-by-lesson
   - ✅ CEFR proficiency levels assigned
   - ✅ AI prompts specified

4. **Output approval checkpoint**:
   ```
   ✅ PHASE 2 COMPLETE: Plan Created

   📋 specs/part-[P]-chapter-[N]/plan.md

   Please review and confirm:
   - ✅ "Plan approved" to proceed to tasks
   - 📝 Feedback to revise plan
   - ❓ Questions for clarification
   ```

5. **Wait for user approval**: Block here until user confirms "✅ Plan approved" OR provides feedback

---

#### PHASE 3: Tasks (Automated + Intelligent) - Triggered After Plan Approval

**THIS PHASE INVOKES `/sp.tasks` AUTOMATICALLY WITH FULL CONTEXT**

1. **Prepare context** (Read approved spec + plan, add validation):
   - Read: `specs/part-[P]-chapter-[N]/spec.md` (learning objectives)
   - Read: `specs/part-[P]-chapter-[N]/plan.md` (lesson structure)
   - Add: Acceptance criteria for each lesson
   - Add: Validation steps (how to test understanding)
   - Add: Implementation checklist (content requirements)

2. **Invoke /sp.tasks with full context**:
   ```
   /sp.tasks [chapter-slug]

   [Full context from spec + plan, acceptance criteria, validation steps]
   ```

3. **Wait for /sp.tasks completion**:
   - ✅ `specs/part-[P]-chapter-[N]/tasks.md` is created
   - ✅ Implementation checklist defined
   - ✅ Validation steps specified

4. **Output completion report**:
   ```
   ✅ ALL DESIGN ARTIFACTS COMPLETE

   📋 specs/part-[P]-chapter-[N]/spec.md
   📋 specs/part-[P]-chapter-[N]/plan.md
   📋 specs/part-[P]-chapter-[N]/tasks.md
   ```

5. **Ask for next step**:
   ```
   Ready to implement?

   A) Implement with content-implementer subagent
   B) Manual implementation (use tasks.md as checklist)
   C) Done for now (keep designs, skip implementation)
   ```

---

#### PHASE 4: Implementation (Optional) - Triggered Only If User Chooses A

**THIS PHASE INVOKES content-implementer subagent IF AND ONLY IF USER CHOOSES OPTION A**

1. **Prepare context** (Read all 3 approved artifacts):
   - Read: `specs/part-[P]-chapter-[N]/spec.md`
   - Read: `specs/part-[P]-chapter-[N]/plan.md`
   - Read: `specs/part-[P]-chapter-[N]/tasks.md`
   - Add: AIDD teaching pattern (What it is → Code → Try → Why it matters)
   - Add: CEFR levels for validation
   - Add: Validation-first approach (test understanding before moving on)

2. **Invoke content-implementer subagent** (Only if user chose Option A):
   ```
   Task(
       subagent_type="content-implementer",
       prompt=prepare_content_implementer_prompt(
           spec, plan, tasks,
           aidd_teaching_pattern=True,
           cefr_levels=True,
           validation_first=True
       )
   )
   ```

3. **Wait for content-implementer completion**:
   - ✅ `docs/part-[P]/chapter-[N]/{01,02,03,04}-lesson-*.md` created
   - ✅ Full AI-Native Learning methodology applied
   - ✅ AI partnership approach emphasized

4. **Invoke validation-auditor** (Automatic validation):
   ```
   Task(
       subagent_type="validation-auditor",
       prompt=f"""
       Validate Chapter {N}: {Title} with special focus on:

       **AI-Native Learning Principles**:
       - 4-step pattern applied (describe intent → explore → validate → learn from errors)
       - AI positioned as co-reasoning partner, not coding assistant
       - Students directing AI, not passive learners

       **Part 4 Appropriate Language**:
       - NO "Specification-Driven Development" terminology (that's Part 5+)
       - Use "describe intent" not "write specifications"
       - AI-Native Learning framing, not professional SDD

       **Lesson Closure Pattern**:
       - ALL lessons end with "Try With AI" section ONLY
       - NO "Key Takeaways", "Summary", "Checklist" after Try With AI
       - Prompt 4 provides cognitive closure

       **Technical Accuracy**:
       - All code runs on Python 3.14+
       - Modern type hints throughout (list[int], dict[str, float], X | None)
       - No security issues, no hardcoded secrets

       **Constitutional Compliance**:
       - All 9 domain skills applied
       - Graduated teaching pattern followed
       - CEFR proficiency levels appropriate
       - Cognitive load within limits

       Output: Validation report with PASS/CONDITIONAL PASS/FAIL verdict
       """
   )
   ```

5. **Apply critical fixes** (if validation identifies issues):
   - Critical issues: MUST fix before proceeding
   - Major issues: SHOULD fix for quality
   - Minor issues: Optional improvements
   - Re-run validation-auditor after fixes

6. **Final report**:
   ```
   ✅ CHAPTER [N] VALIDATED AND COMPLETE

   📚 Lessons created: docs/part-[P]/chapter-[N]/
   📋 Validation report: VALIDATION_REPORT_CHAPTER_[N].md

   Next steps:
   - Review validation report
   - Test lessons interactively
   - Prepare for publication
   - Commit to git
   ```

---

### CRITICAL EXECUTION RULES

1. **Sequential Invocation**: Phases execute in order (0 → 1 → 2 → 3 → 4), never out of order
2. **Automatic Chaining**: Each phase automatically invokes the next command (no "ask user first")
3. **Approval Gates Only Between Phases**: User approves AFTER each phase completes, BEFORE next phase starts
4. **Context Preservation**: Each phase reads prior phase outputs and passes them forward
5. **Vertical Intelligence Embedded**: EVERY command invocation includes AIDD principles, teaching patterns, cognitive load rules
6. **Ruthless Filtering**: Materials from future chapters are SKIPPED, not extracted
7. **No User Override**: User intent (audience, scope, outcome) is NEVER overridden, only honored
8. **Feature Branch Creation**: Automatic checkout of feature branch in PHASE 0, before any other work
9. **All 3 Artifacts Required**: Spec, Plan, and Tasks must exist before implementation can proceed

---

## CRITICAL VALIDATION (Before Each Phase)

**PHASE 1 Validation** (before `/sp.specify`):
- ✅ Chapter number valid (12-29, Part 4 only)
- ✅ Chapter title matches `chapter-index.md`
- ✅ User's audience answer captured
- ✅ User's scope answer captured
- ✅ User's outcome answer captured
- ✅ Context will be ruthlessly filtered (skip future chapters)
- ✅ AI-Native Learning principles will be applied (NOT formal SDD)

**PHASE 2 Validation** (before `/sp.plan`):
- ✅ spec.md was created successfully
- ✅ Concept count ≤ tier limit (5/7/10 based on chapter range)
- ✅ No forward references (Chapters 30+ or SDD terminology)
- ✅ AI-Native Learning framing used (not formal SDD)
- ✅ Only Chapters 1-N are prerequisites
- ✅ Teaching pattern respected (Book → AI Companion → AI Orchestration)
- ✅ Skills proficiency mapping will be applied

**PHASE 3 Validation** (before `/sp.tasks`):
- ✅ plan.md was created successfully
- ✅ Lessons match spec's learning objectives
- ✅ Proficiency levels assigned (CEFR A1/A2/B1)
- ✅ Cognitive load validated (concepts per lesson within limits)
- ✅ AI prompts specified for each lesson (4 prompts, progressive)
- ✅ Validation points defined
- ✅ Lesson closure pattern specified (Try With AI ONLY)

**PHASE 4 Validation** (before content-implementer):
- ✅ All 3 design files exist and are valid
- ✅ User chose implementation option
- ✅ Context filtered ruthlessly (no future chapters)
- ✅ AI-Native Learning principles embedded
- ✅ Graduated teaching pattern clear
- ✅ Ready for lesson content creation

**PHASE 4 Post-Implementation Validation** (validation-auditor):
- ✅ All lessons implement AI-Native Learning pattern
- ✅ No SDD terminology used inappropriately
- ✅ Lesson closure pattern followed (Try With AI ONLY)
- ✅ Code quality validated (Python 3.14+, type hints)
- ✅ CEFR proficiency levels appropriate
- ✅ Constitutional compliance verified
- ✅ **NEW: Pedagogical Ordering Compliance (CRITICAL)**
  - No forward references within chapter (Lesson N only uses concepts from Lessons 1 to N)
  - All methods/functions introduced before first use
  - Built-in functions (len, type, isinstance) distinguished from methods (.upper, .split)
  - Every new concept has explicit introduction ("what it is, what it does, why it matters")

---

## WHAT GETS CREATED

**By End of PHASE 3** (mandatory):
```
specs/part-4-chapter-[N]/
  ├── spec.md       (What students learn + AI-Native Learning principles)
  ├── plan.md       (How we teach it, lesson-by-lesson + CEFR levels + skills mapping)
  └── tasks.md      (Implementation checklist + validation)
```

**By End of PHASE 4** (if Option A chosen):
```
book-source/docs/04-Part-4-Python-Fundamentals/[N]-[chapter-name]/
  ├── readme.md                    (Chapter overview and navigation)
  ├── 01-[lesson-name].md          (Lesson 1)
  ├── 02-[lesson-name].md          (Lesson 2)
  ├── 03-[lesson-name].md          (Lesson 3)
  ├── 04-[lesson-name].md          (Lesson 4, or more based on plan)
  └── 05-[capstone-name].md        (Optional capstone lesson)

VALIDATION_REPORT_CHAPTER_[N].md   (Technical review results)
```

---

## EXAMPLE EXECUTION (What User Sees)

```
> /sp.python-chapter 14

⏺ PHASE 0: Intelligent Context Discovery

📖 Reading authoritative sources...
  ✓ Constitution: Target audience, philosophy, principles
  ✓ Chapter Index: Chapter 14 = "Data Types" (Part 4)
  ✓ Skills Library: 13 skills available
  ✓ Context: Found existing materials in context/13_chap12_to_29_specs/

🧠 Deriving chapter intelligence...
  ✓ Chapter: 14 - "Data Types"
  ✓ Part: 4 (Python Fundamentals)
  ✓ Complexity Tier: Beginner (Chapters 12-16)
  ✓ CEFR Range: A1-A2 (max 5 concepts/lesson)
  ✓ Prerequisites: Chapters 1-13
  ✓ Audience: Aspiring/Professional/Founders (graduated)
  ✓ Learning Pattern: AI-Native Learning (NOT formal SDD)

🤔 Analyzing what needs clarification...
  → Existing context materials found

  Q: Use existing context or start from scratch?
  [User: Use existing but adapt for AI-Native Learning emphasis]

  → Chapter title "Data Types" is broad (could be 10+ topics)

  Q: Which specific data types should we emphasize?
  [User: Core types only - int, float, str, bool, None + type hints]

  → Checking if capstone needed...

  Q: Should students BUILD something hands-on?
  [User: Yes - Interactive Type Explorer program]

✅ Intelligence gathered (2 automatic derivations + 3 targeted questions). Moving to PHASE 1...

---

⏺ PHASE 1: Specification + Clarification Gate

Invoking: /sp.specify with derived intelligence...
Applying: AI-Native Learning principles, teaching patterns, cognitive load limits

[/sp.specify generates spec.md]

📋 Spec created: specs/part-4-chapter-14/spec.md

---

🔍 Running /sp.clarify to identify underspecified areas...

Analyzing spec for ambiguities...
  ✓ Learning objectives: Clear and measurable
  ✓ Prerequisites: Explicitly listed
  ⚠️ Scope clarity: "Data Types" could mean 10+ topics
  ⚠️ Code examples: Quantity not specified
  ⚠️ Capstone: Build artifact not defined

Asking targeted clarifications (3 questions):

Q1: Which specific data types should we cover?
    (Found: int, float, str, bool, None, list, dict, tuple, set)
[User: Focus on core 5 only - int, float, str, bool, None. Collections are Chapter 18-19]

Q2: How many code examples per lesson?
[User: 3-6 examples per lesson, progressive complexity]

Q3: Should students BUILD a hands-on project?
[User: Yes - Interactive Type Explorer program demonstrating all core types]

✅ Spec updated with clarifications

---

Please review the clarified specification:
- Does scope now match chapter title anchor "Data Types"?
- Are learning objectives achievable in 3.5-4 hours?
- Is capstone project appropriate for A1-A2 learners?

Reply with: "✅ Spec approved" or provide feedback

[User: ✅ Spec approved]

✅ Spec approved. Moving to PHASE 2...

---

⏺ PHASE 2: Planning + ADR Gate

Invoking: /sp.plan with approved spec...
Applying: Lesson progression, CEFR levels, AI prompts, skills-proficiency-mapper

[/sp.plan generates plan.md with skills metadata]

📋 Plan created: specs/part-4-chapter-14/plan.md
  - 5 lessons (4 foundational + 1 capstone)
  - CEFR: A2-B1 progression
  - Skills mapped to each lesson
  - 4 "Try With AI" prompts per lesson

---

🏛️ Running /sp.adr to detect architectural decisions...

Analyzing plan for significant decisions...
  ✓ Lesson structure: Standard 4-prompt format (no ADR needed - established pattern)
  ✓ CEFR progression: A2 → A2-B1 → B1 (standard tier)
  📋 Pedagogical approach detected: "Collections awareness in Lesson 4 vs deep dive later"

📋 Architectural decision detected:
   "Teaching collections as awareness-only in Chapter 14 vs comprehensive coverage"

   Rationale: Collections (list, dict, tuple, set) are complex (7+ concepts each).
   Chapter 14 scope = core types only. Deep dive deferred to Chapters 18-19.

   Decision: Lesson 4 teaches "what collections exist" + basic syntax only.
             No iteration, comprehension, or methods until Ch 18-19.

   Document this decision with ADR? (y/n)

[User: y]

Creating ADR: history/adr/014-collections-awareness-pattern.md
✅ ADR created and linked to plan.md

---

Please review the plan + ADR:
- Are 5 lessons appropriate structure?
- Is collections awareness pattern justified?
- Are CEFR levels and cognitive load appropriate?

Reply with: "✅ Plan approved" or provide feedback

[User: ✅ Plan approved]

✅ Plan approved. Moving to PHASE 3...

---

⏺ PHASE 3: Tasks + Analysis Gate

Invoking: /sp.tasks with spec + plan...
Applying: Validation points, acceptance criteria, lesson structure

[/sp.tasks generates tasks.md]

📋 Tasks created: specs/part-4-chapter-14/tasks.md
  - 58 tasks across 8 phases
  - Organized by lesson (not user story)
  - Code testing tasks included

---

🔬 Running /sp.analyze for cross-artifact consistency...

Analyzing spec ↔ plan ↔ tasks alignment...

**Traceability Matrix**:
  ✓ Learning Objective 1 → Lesson 1 → Tasks T004-T010 ✓
  ✓ Learning Objective 2 → Lesson 2 → Tasks T011-T018 ✓
  ✓ Learning Objective 3 → Lesson 3 → Tasks T019-T026 ✓
  ✓ Learning Objective 4 → Lesson 4 → Tasks T027-T035 ✓
  ✓ Learning Objective 5 → Lesson 5 → Tasks T036-T043 ✓

**Scope Validation**:
  ✓ All spec concepts have corresponding lessons
  ✓ All lessons have implementation tasks
  ✓ No orphaned tasks (all map to spec/plan)

**Consistency Checks**:
  ✓ CEFR levels consistent (spec A2-B1 = plan A2-B1 = tasks complexity)
  ✓ Cognitive load respected (max 7 concepts per lesson in tasks)
  ✓ Lesson closure pattern specified in tasks (Try With AI ONLY)
  ✓ Code testing tasks for all examples

**Risk Analysis**:
  ⚠️ MINOR: Lesson 5 (capstone) has 8 tasks - ensure not overwhelming
  ✓ Mitigation: Capstone is integration (reuses L1-4 concepts, not 8 NEW concepts)

📊 Analysis Report: All critical checks passed. 1 minor advisory noted.

---

Please review tasks.md + analysis report:
- Are 58 tasks reasonable for 5 lessons?
- Is spec → plan → tasks traceability clear?
- Any critical consistency issues?

Reply with: "✅ Tasks approved" or provide feedback

[User: ✅ Tasks approved]

✅ Tasks approved. Moving to PHASE 4...

---

⏺ PHASE 4: Implementation + Technical Review Gate

All design artifacts approved:
  ✅ specs/part-4-chapter-14/spec.md (clarified)
  ✅ specs/part-4-chapter-14/plan.md (ADR documented)
  ✅ specs/part-4-chapter-14/tasks.md (analyzed)

Invoking: /sp.implement 014-data-types

📚 Implementing lessons with parallel team strategy...
  → Lessons 1-4: Parallel (4 content-implementer subagents)
  → Lesson 5: Sequential (capstone after L1-4 complete)

[content-implementer subagents working...]

✅ All 5 lessons created:
  ✓ book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/readme.md
  ✓ book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/01-variables-and-type-hints.md
  ✓ book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/02-integers-and-floats.md
  ✓ book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/03-strings-and-booleans.md
  ✓ book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/04-collections-awareness.md
  ✓ book-source/docs/04-Part-4-Python-Fundamentals/14-data-types/05-type-explorer-capstone.md

---

🔍 Invoking validation-auditor for validation...

Validating AI-Native Learning compliance...
  ✓ 4-step pattern applied (describe → explore → validate → learn)
  ✓ AI positioned as co-reasoning partner
  ✓ Part 4 appropriate language (no SDD terminology)

Validating lesson closure pattern...
  ✓ Lessons 1-4 end with "Try With AI" ONLY
  ⚠️ Lesson 5: Has "Capstone Completion Checklist" after "Try With AI" (CRITICAL VIOLATION)

Validating code quality...
  ✓ All 19 code examples run on Python 3.14+
  ✓ Modern type hints throughout
  ✓ No security issues

Validating constitutional compliance...
  ✓ All 9 domain skills applied
  ✓ Graduated teaching pattern followed
  ✓ CEFR proficiency appropriate

📋 Verdict: CONDITIONAL PASS (1 critical issue)

---

⚠️ CRITICAL ISSUE DETECTED: Lesson 5 closure pattern violation

Applying automatic fix...
  → Removing lines 559-575 (Capstone Completion Checklist section)
  → Lesson now ends at line 555 with "Try With AI" section

✅ Fix applied

---

🔍 Re-running validation-auditor...

✅ PASS: All requirements met
  - Lesson closure pattern compliant
  - AI-Native Learning principles excellent
  - Code quality 100% (19/19 examples pass)
  - Constitutional alignment verified

📋 Validation Report: VALIDATION_REPORT_CHAPTER_14.md

---

✅ WORKFLOW COMPLETE

Chapter 14 production-ready:
  ✅ Specification (clarified with /sp.clarify)
  ✅ Planning (ADR documented with /sp.adr)
  ✅ Tasks (consistency validated with /sp.analyze)
  ✅ Implementation (5 lessons, 94 KB content)
  ✅ Technical Review (PASS after critical fix)

Files created: 6 lessons + 1 validation report
Quality gates: 4/4 passed (clarify, ADR, analyze, technical-review)

Next: Commit to git → Create PR → Publish
```

---

## CRITICAL SUCCESS FACTORS

1. **Automatic Invocation**: `/sp.specify`, `/sp.plan`, `/sp.tasks` must be invoked automatically via SlashCommand tool with full intelligence context

2. **Vertical Intelligence Preserved**: Every phase applies AIDD principles, teaching patterns, pedagogical design, and chapter boundary awareness

3. **Approval Gates**: User must explicitly approve each phase ("✅ Approved") before proceeding to next

4. **Context Preservation**: Each phase receives full context from all previous phases + vertical intelligence

5. **Ruthless Filtering**: Context extraction must skip any concepts from future chapters, even if present in materials

6. **User Authority**: User's answers to 4 questions are final — never override with assumptions

7. **Compliance**: Every phase validates against acceptance criteria before proceeding

8. **Teaching Quality**: Intelligence flows through all 4 phases, not just documentation

---

## REFERENCES

- **Chapter Index**: `specs/book/chapter-index.md` (Part 4 Chapters: 12-29)
- **Constitution**: `.specify/memory/constitution.md` (AI-Native Learning principles, domain skills, graduated teaching pattern)
- **Skills Library**: `.claude/skills/` (skills-proficiency-mapper, learning-objectives, concept-scaffolding, etc.)
- **Context Materials**:
  - `context/13_chap12_to_29_specs/` (legacy structure)
  - `context/part-4-python/` (preferred structure)

---

## ONE COMMAND. FULL INTELLIGENCE. COMPLETE WORKFLOW WITH QUALITY GATES.

Run `/sp.python-chapter [N]` and the system executes this opinionated workflow:

**PHASE 0: Intelligent Context Discovery**
✅ Reads constitution + chapter-index + skills (automatic intelligence derivation)
✅ Asks 0-3 targeted questions (only when genuinely ambiguous)

**PHASE 1: Specification + Clarification Gate**
✅ `/sp.specify` → Creates spec.md
✅ `/sp.clarify` → Identifies underspecified areas, asks up to 5 clarifications, updates spec
✅ Human review → Approval gate

**PHASE 2: Planning + ADR Gate**
✅ `/sp.plan` → Creates plan.md with CEFR levels, skills mapping
✅ `/sp.adr` → Detects architectural decisions, suggests documentation (waits for user consent)
✅ Human review → Approval gate

**PHASE 3: Tasks + Analysis Gate**
✅ `/sp.tasks` → Creates tasks.md with acceptance criteria
✅ `/sp.analyze` → Cross-artifact consistency check (spec ↔ plan ↔ tasks alignment)
✅ Human review → Approval gate

**PHASE 4: Implementation + Technical Review Gate**
✅ `/sp.implement` → content-implementer creates all lessons (parallel + sequential strategy)
✅ `validation-auditor` → Validates AI-Native Learning compliance, code quality, lesson closure
✅ Auto-fix critical issues → Re-validate until PASS
✅ Human review → Final approval

**Result: High-quality, AI-Native Learning-centered Python chapters with built-in quality assurance.**

---

**Note**: For PHR (Prompt History Record) creation after command completion, see constitution for instructions.
