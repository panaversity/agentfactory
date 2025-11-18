# /sp.constitution-sync: Constitutional Alignment Through Intelligent Partnership

**Version**: 3.0.0 (Context Engineering: 95% Quality Target)
**Created**: 2025-01-10
**Updated**: 2025-01-12
**Philosophy**: Constitution-grounded, judgment-driven, quality-preserving, partnership-focused

---

## Purpose

Bring existing chapter content into **authentic alignment** with constitutional values—not mechanical compliance, but **embodiment of principles**.

**Core Intent**: Ensure content demonstrates:
- **Three-Role AI Partnership** (Principle 18): AI as Teacher, Student, and Co-Worker
- **Co-Learning** (Philosophy #2): Bidirectional learning, not passive consumption
- **"Specs Are the New Syntax"** (Philosophy #3): Specification-writing as primary skill
- **Validation-First** (Philosophy #5): Never trust, always verify

**Key Innovation**: Intelligent per-lesson decisions based on **constitutional understanding** and **content quality judgment**, not formulas or checklists.

---

## Syntax

```bash
/sp.constitution-sync [chapter-number]
```

**Examples**:
```bash
/sp.constitution-sync 1     # Sync Chapter 1 (conceptual)
/sp.constitution-sync 14    # Sync Chapter 14 (code-focused)
```

---

## Distinction: Error Analysis vs Constitution-Sync

| Aspect | `/sp.error-analysis` | `/sp.constitution-sync` |
|--------|---------------------|------------------------|
| **Purpose** | **Diagnose** workflow issues | **Align** content with constitutional values |
| **Trigger** | After workflow execution (reactive) | After constitution change (proactive) |
| **Input** | Executed artifacts (traces) | Constitution + existing chapters |
| **Output** | Report + recommendations | Aligned chapters |
| **Action** | Read-only analysis | Write operations (edits/regen) |
| **Timing** | Post-mortem | Maintenance |
| **Focus** | Process issues | Constitutional embodiment |

**Relationship**: Error analysis can debug constitution-sync itself (if sync has issues, run error analysis on sync execution to diagnose).

---

## I. CONSTITUTIONAL GROUNDING

### The Constitution as Source of Truth

**Before any assessment or intervention**, deeply understand constitutional intent.

**Reference**: `.specify/memory/constitution.md` (latest version)

#### What to Internalize

**18 Core Principles** (focus on most relevant):
- **Principle 13**: Graduated Teaching Pattern (Book → AI Companion → AI Orchestration)
- **Principle 18**: Three-Role AI Partnership (AI as Teacher/Student/Co-Worker)
- **Principle 6**: Consistent Structure Across All Chapters
- **Principle 3**: Specification-First Development
- **Principle 2**: AI as Co-Learning Partner
- **Principle 5**: Validation-Before-Trust

**8 Core Philosophies**:
- **Co-Learning**: Bidirectional learning (human ↔ AI refine each other)
- **Evals-First**: Define success criteria before specs
- **Spec-Driven**: Specification is the new syntax
- **Validation-First**: Never trust AI output without verification

**Nine Pillars**: AI CLI, Markdown, MCP, AI-First IDEs, Cross-Platform, TDD, SDD, Composable Skills, Cloud-Native

---

### 🎯 Output Style Requirements: THE AUTHORITATIVE SOURCE

**CRITICAL HIERARCHY** (resolving conflicts):
1. **Constitution** (`.specify/memory/constitution.md`) = Strategic governance, principles, philosophies
2. **Output Styles** (`.claude/output-styles/lesson.md`) = **Format specifications, structure, CoLearning standards**
3. **This Command** = Workflow orchestration, references above sources

**When conflicts exist**: Constitution > Output Styles > This Command
**For formatting/structure questions**: **Output Styles are authoritative**

**Constitution Principle 6** states:
> "All lessons follow identical teaching structure (documented in `.claude/output-styles/lesson.md`)"

**Therefore**: Before making any formatting or structural decisions, **READ `.claude/output-styles/lesson.md` lines 302-435** for complete CoLearning element standards.

---

### CoLearning Element Standards (From Output Style)

**AUTHORITATIVE SOURCE**: `.claude/output-styles/lesson.md` (lines 302-435)

**Three CoLearning Element Types**:
- 💬 **AI Colearning Prompts**: Exploration-focused questions encouraging deeper understanding
- 🎓 **Expert Insights**: Strategic depth and pedagogical perspective ("Syntax is cheap, semantics is gold")
- 🤝 **Practice Exercises**: Hands-on collaborative practice with AI partnership

**🚨 QUANTITY GUIDELINES (per lesson)**:

**Standard Pattern**: **1 element of each type per lesson** (💬 + 🎓 + 🤝 = 3 total)

| Element Type | Count | Purpose | Quality Check |
|--------------|-------|---------|---------------|
| 💬 **AI Colearning Prompt** | 1 per lesson | Encourage exploration of concept depth | Specific to lesson, encourages "why/what-if" thinking |
| 🎓 **Expert Insight** | 1 per lesson | Provide strategic depth and pedagogical perspective | Demonstrates "syntax is cheap, semantics is gold" |
| 🤝 **Practice Exercise** | 1 per lesson | Hands-on collaborative practice with AI | Practices specification-writing and validation |

**🎯 QUALITY MANTRA FOR PROFESSIONAL LEARNERS**:
- **1 excellent element of each type > multiple mediocre variations**
- **Each element must earn its place** (specific, insightful, actionable)
- **Remove generic or redundant content ruthlessly**
- **"Would I want to read this if I were the student?" = litmus test**
- **Consistency matters**: Same pattern across lessons creates predictable learning experience

**CRITICAL**: **1 of each type = 3 total per lesson**. This provides **consistent structure** while respecting professional learners' time and attention. **Quality over quantity.**

---

### CoLearning Element Formats (From Output Style)

**CRITICAL**: Use **exact formats** from `.claude/output-styles/lesson.md`. No variations allowed.

**💬 AI Colearning Prompt Format**:
```markdown
#### 💬 AI Colearning Prompt
> "Explain how [concept] works under the hood."
```

**🎓 Expert Insight Format**:
```markdown
#### 🎓 Expert Insight
> In AI-native development, [reframe from memorization to understanding].
```

**🤝 Practice Exercise Format**:
```markdown
#### 🤝 Practice Exercise

> **Ask your AI**: "[Specification of what to create] Then explain [conceptual aspect] step-by-step."

**Expected Outcome**: [What student should understand after AI response]
```

**❌ DO NOT USE** (incorrect formats):
- `> **Explore with your AI**: "..."` (wrong wording)
- `**Quick Test**: Ask your AI: "..."` (no prefix outside blockquote)
- `Ask your AI Co-Teacher:` followed by blockquote (no separate intro line)
- `Ask your AI:` on separate line before blockquote (outdated format)
- Variations in blockquote style or structure

**✅ ALWAYS USE** (correct format):
- Single blockquote with bold prefix: `> **Ask your AI**: "..."`
- "Expected Outcome" in all 🤝 exercises
- No separate intro line before blockquote

---

### Lesson Closure Requirements

**From Constitution & Output Style**:
- **FINAL SECTION**: "Try With AI" (must be last section in every lesson)
- **NO POST-SECTIONS**: Never add "Key Takeaways," "What's Next," "Summary," "Recap," or "Completion Checklist" after "Try With AI"

**Why this matters**: "Try With AI" is the practice-integration point—nothing should dilute or distract from it.

---

### Why Alignment, Not Compliance

**We're not checking boxes—we're ensuring content embodies constitutional values.**

**Questions to guide assessment**:
- Does content teach **collaboration with AI**, not just tool usage?
- Does it demonstrate **Three-Role Partnership** authentically?
- Does it emphasize **specs over syntax**?
- Does it **validate alongside generation**?
- Does it encourage **exploration** over prescription?

---

## II. INTELLIGENT ASSESSMENT FRAMEWORK

### Phase 1: Constitutional Context Discovery

**Your Role**: AI as **Student** — Learn what the constitution requires before evaluating content.

#### Step 1.1: Read Constitution & Output Style

**CRITICAL**: Read BOTH sources before assessing content.

```bash
# 1. Read constitution for principles and philosophies
Read .specify/memory/constitution.md

# 2. Read output style for formatting and structure standards
Read .claude/output-styles/lesson.md (lines 302-435 for CoLearning)
```

**Extract Understanding** (not just data):
- **Version**: What changed in latest constitution version? (check HTML comment at top)
- **Sync Impact**: What's the impact of recent changes? (from Sync Impact Report)
- **Applicable Principles**: Which of the 18 principles apply to this chapter type?
- **Target Level**: What's the proficiency level? (A1-beginner, A2-elementary, B1-intermediate, B2-advanced, C1-professional)
- **CoLearning Pattern**: 1 of each type per lesson (💬 + 🎓 + 🤝 = 3 total)
- **CoLearning Formats**: What are the exact markdown formats? (from output style)
- **Constitutional Intent**: Why do these rules exist? (understand spirit, not just letter)

#### Step 1.2: Categorize by Impact and Context

**High-Impact Requirements** (must be present in ALL lessons):
- CoLearning elements (💬🎓🤝) demonstrating Three-Role Partnership
- CoLearning element pattern: **1 of each type** (💬 + 🎓 + 🤝 = 3 total per lesson)
- CoLearning element formats match output style exactly (no variations)
- Lesson closure pattern ("Try With AI" is final section, no post-sections)
- No forward references (pedagogical ordering: concepts introduced before use)
- Conversational, exploration-focused tone (not documentation style)

**Medium-Impact Requirements** (context-dependent):
- Graduated Teaching Pattern (relevant for code-heavy chapters)
- Cognitive load management (A1: max 5 concepts, A2: max 7, B1: max 10)
- Specification-first pedagogy (for code lessons: show spec → prompt → code → validation)

**Low-Impact Requirements** (technical standards):
- Python 3.13+ (for code lessons)
- Reading level baseline (Grade 7-8 accessibility)
- Type hints (technical correctness)

#### Step 1.3: Create Constitutional Lens

**Output** (internal understanding, not mechanical checklist):

```markdown
## Constitutional Lens for Chapter [N]

**Chapter Type**: [Conceptual / Code-focused / Mixed]
**Target Level**: [A1-Beginner / A2-Elementary / B1-Intermediate / B2-Advanced / C1-Professional]
**CoLearning Element Pattern**: 1 of each type per lesson (💬 + 🎓 + 🤝 = 3 total)
**Primary Principles**: [List 3-5 most relevant of the 18 principles]

**What "Good" Looks Like**:
- [Describe ideal embodiment of constitutional values for THIS chapter]
- [Not generic—specific to chapter content and audience]

**Common Gaps to Watch For**:
- [Based on chapter type, predict likely violations]
- [Example: Conceptual chapters often missing hands-on challenges]
- [Example: Professional chapters often over-stuffed with CoLearning elements]
```

---

### Phase 2: Content Assessment with Judgment

**Your Role**: AI as **Teacher** — Diagnose not just what's missing, but whether content embodies constitutional intent.

#### Step 2.1: Locate Chapter Artifacts

**Find spec/plan (if they exist)**:
```bash
# Typical locations
specs/part-N-chapter-M/spec.md
specs/part-N-chapter-M/plan.md
```

**Find lesson files**:
```bash
# Typical pattern
book-source/docs/NN-Part-N/MM-chapter-title/*.md
```

**Note**: Not all chapters have specs/plans (especially Part 1 conceptual). Assess lessons directly if artifacts missing.

#### Step 2.2: For Each Lesson - Read with Constitutional Lens

**Not**: Count elements mechanically
**But**: Judge whether content demonstrates constitutional values

**Read lesson file**:
```bash
Read [lesson-file-path]
```

**Questions to Ask** (in order of importance):

##### 1. CoLearning Elements (High Impact)

**Quantity Assessment** (Standard Pattern):
- **Expected**: **1 element of each type** (💬 + 🎓 + 🤝 = 3 total per lesson)
- **Count by type**:
  - 💬 AI Colearning Prompts: Expected 1
  - 🎓 Expert Insights: Expected 1
  - 🤝 Practice Exercises: Expected 1
- **🚨 CONSISTENCY CHECK**:
  - **Missing types**: Flag if any type is absent (lesson should have all 3 types)
  - **Over-stuffing**: Flag if lesson has 2+ of any single type (unnecessary duplication)
  - **Total count**: Should be exactly 3 (1+1+1), not more, not less

**Format Assessment** (exact match required):
- Do 💬 prompts use format: `#### 💬 AI Colearning Prompt\n> "..."`?
- Do 🎓 insights use format: `#### 🎓 Expert Insight\n> In AI-native development, ...`?
- Do 🤝 exercises use format: `#### 🤝 Practice Exercise\n\n> **Ask your AI**: "..."\n\n**Expected Outcome**: ...`?
- **🚨 FORMAT DRIFT CHECK**: Flag any variations (bold prefixes, different wording, missing components)

**Quality Assessment** (more important than quantity):
- Do 💬 prompts encourage **exploration** ("What happens if..." / "Why does X instead of Y?") or just "Ask AI to write X"?
- Do 🎓 insights provide **strategic depth** and pedagogical perspective (why this matters, non-obvious implications, "syntax is cheap—semantics is gold") or just restate content?
- Do 🤝 exercises practice **collaborative learning** (iteration, validation, partnership, "Specification → AI Generation → Explanation → Understanding") or passive copying?
- **🚨 GENERIC CONTENT CHECK**: Are elements specific to lesson content, or could they apply to any lesson? (Generic = low quality)

##### 2. Three-Role AI Partnership (High Impact)

**AI as Teacher**:
- Does content show AI suggesting patterns student doesn't know yet?
- Are prompts structured as "Ask AI to teach/explain" not just "Ask AI to generate"?

**AI as Student**:
- Does content show AI adapting to student specifications/feedback?
- Are examples of iteration (student → AI → student refines → AI adapts)?

**AI as Co-Worker**:
- Is language collaborative ("Let's explore together") vs. command-driven ("Use AI to...")?
- Does content emphasize partnership over tool usage?

##### 3. Lesson Closure Pattern (High Impact)

**Structural requirement**:
- Does lesson end with "Try With AI" section? (must be final section)
- Are there ANY sections after "Try With AI"? (❌ VIOLATION if yes)
- Sections to flag: "Key Takeaways," "What's Next," "Summary," "Recap," "Completion Checklist"

**Why this matters**: "Try With AI" is the practice-integration point—nothing should dilute or distract from it.

##### 4. Pedagogical Ordering (High Impact)

**No forward references**:
- Are concepts introduced before use?
- Are there references to future chapters/lessons without explanation?

**Critical judgment needed**:
- Is a "violation" actually a problem, or is context provided?
- Example: "We'll learn functions in Chapter 5" = ✅ Acceptable (preview with context)
- Example: Using `name.upper()` before teaching methods = ❌ Violation (blocks learning)

##### 5. Graduated Teaching Pattern (Medium Impact)

**Relevant for code lessons**:
- **Tier 1 (Book teaches)**: Are foundational concepts explained directly?
- **Tier 2 (AI Companion)**: Is complex syntax delegated to AI?
- **Tier 3 (AI Orchestration)**: Are scaling operations automated?

**Not all lessons need all tiers**—judge contextually.

##### 6. Cognitive Load (Medium Impact)

**Count new concepts** introduced in lesson:
- A1 (beginner): max 5 new concepts
- A2 (elementary): max 7 new concepts
- B1 (intermediate): max 10 new concepts

**Is limit respected?** If over limit, is pacing appropriate?

##### 7. Tone & Style (Medium Impact)

**Conversational vs. documentation**:
- Is tone conversational ("Let's explore") or documentary ("Functions are...")?
- Is exploration encouraged ("What happens if...") over prescription ("Do this")?

**AI partnership emphasis**:
- Does content treat AI as partner or tool?
- Is language collaborative or transactional?

##### 8. Specification-First Pedagogy (Medium Impact, code lessons)

**For code examples**:
- Do they show: spec → AI prompt → code → validation?
- Or just: "Here's code, copy it"?

**"Specs Are the New Syntax"** emphasis:
- Is specification-writing treated as primary skill?
- Is value positioned as "articulating intent clearly" not "typing syntax fast"?

#### Step 2.3: Assess Content Quality Independently

**Separate Constitutional Alignment from Content Quality**:

**Constitutional Alignment**: [Excellent / Good / Needs Work / Poor]
- Does it embody values authentically?

**Content Quality**: [Excellent / Good / Needs Improvement]
- Is writing clear and engaging?
- Are examples effective?
- Is narrative flow smooth?
- Is teaching effective (independent of constitutional alignment)?

**Why separate?**
- **Excellent content + gaps** → Surgical edit (preserve quality, add elements)
- **Good content + mixed issues** → Enhanced regeneration (preserve good parts, fix problems)
- **Poor content + violations** → Full regeneration (opportunity to improve both)

#### Step 2.4: Qualitative Assessment Output

```markdown
### Lesson Assessment: [Lesson N - Title]

**Constitutional Alignment**: [Excellent / Good / Needs Work / Poor]

**Strengths**:
- [What embodies constitutional values well?]
- [Specific examples: "Expert Insight on line 145 demonstrates Three-Role Partnership clearly"]

**Gaps**:
- [What's missing or misaligned?]
- [Specific, not generic: "Has 7 CoLearning elements (over-stuffed for B1 level, expected 4-6)"]
- [Example: "💬 Prompts use incorrect format (bold prefix 'Explore with your AI:' instead of clean blockquote)"]
- [Example: "Prompts present but don't encourage exploration—they're task-focused ('Ask AI to write X')"]

**Content Quality**: [Excellent / Good / Needs Improvement]
- Writing clarity: [assessment]
- Examples effectiveness: [assessment]
- Narrative flow: [assessment]

**Recommended Intervention**: [Surgical Edit / Enhanced Regeneration / Full Regeneration / No Change]

**Rationale**: [Why this intervention preserves quality while achieving alignment]
- [Example: "Surgical edit preserves excellent narrative and examples, removes 3 generic CoLearning elements, standardizes remaining 4 to output style format"]
```

---

### Phase 3: Intelligent Intervention Decisions

**Your Role**: AI as **Co-Worker** — Partner with user to decide best approach for each lesson.

#### Decision Framework: Judgment, Not Formula

##### Option 1: Surgical Edit (Preserve excellent content, add/fix/remove elements)

**When to choose**:
- ✅ Content quality is **excellent** (clear writing, effective teaching, good examples)
- ✅ Gaps are **structural** (missing/excessive CoLearning elements, format drift, post-sections to remove)
- ✅ Tone and partnership language already appropriate
- ✅ Can fix elements naturally without disrupting flow
- ✅ No pedagogical ordering violations

**What this involves**:
- **Add** missing CoLearning elements at natural break points (if below target count)
- **Remove** excessive/generic CoLearning elements (if above target count or low quality)
- **Standardize** existing elements to output style format (fix format drift)
- **Remove** post-sections (e.g., "What's Next" after "Try With AI")
- **Enhance** existing content with partnership language (minimal rewording)
- **Validate**: Ensure changes feel natural, not forced

**Time**: 10-15 minutes per lesson

**Example scenario**:
- Lesson with excellent narrative and examples
- Has 5 CoLearning elements (over-stuffed: 2 💬, 2 🎓, 1 🤝)
- 1 💬 prompt is generic, 1 🎓 insight restates obvious content
- Good conversational tone already
- **Decision**: Remove 1 generic 💬 and 1 low-quality 🎓, keep 1 of each type (3 total), standardize formats, preserve narrative

##### Option 2: Enhanced Regeneration (Preserve good parts, regenerate problem areas)

**When to choose**:
- ✅ Content quality is **good** (worth preserving examples, explanations, analogies)
- ✅ Gaps are **mixed** (structural + tone/ordering/partnership issues)
- ✅ Core content solid but needs constitutional framing
- ✅ Can extract and reuse quality content with new narrative

**What this involves**:
- **Extract** excellent examples, explanations, analogies, code samples
- **Identify** sections needing rewrite (documentation tone, missing partnership, ordering issues)
- **Regenerate** with constitution + output style + preserved content:
  - Use content-implementer skill
  - Provide extracted content as "preserve these examples"
  - **Reference `.claude/output-styles/lesson.md` explicitly** for CoLearning element standards
  - Emphasize constitutional framing (Three-Role Partnership, exploration focus)
  - Add CoLearning elements throughout (not just inserted afterward)
  - **Follow proficiency tier quantity guidelines** (avoid over-stuffing)
- **Validate**: Does it preserve quality? Does it embody constitution? Natural flow? Correct element count and format?

**Time**: 20-30 minutes per lesson

**Example scenario**:
- Lesson with solid code examples (preserve)
- Documentation tone throughout (regenerate narrative)
- Missing Three-Role Partnership framework (add via regeneration)
- **Decision**: Extract examples, regenerate narrative with constitutional framing, 1 of each CoLearning type (3 total) in output style format

##### Option 3: Full Regeneration (Start fresh with constitutional intent)

**When to choose**:
- ✅ Content quality **needs improvement** OR
- ✅ **Critical pedagogical violations** (forward references blocking learning) OR
- ✅ **Fundamental misalignment** with constitutional values (tool-driven vs. partnership) OR
- ✅ **Spec/plan changed** significantly (lesson outdated)

**What this involves**:
- **Review spec/plan** (if they exist):
  - What are learning objectives? (from spec.md)
  - What's the lesson structure? (from plan.md)
  - What proficiency level? (CEFR metadata)
- **Ground in constitution AND output style**:
  - Which principles apply to this content?
  - How should Three-Role Partnership manifest here specifically?
  - What does "Specs Are the New Syntax" mean in this context?
  - **What's the CoLearning element count for this proficiency tier?** (from output style)
  - **What are the exact CoLearning element formats?** (from output style)
- **Generate** with constitutional lens:
  - Use content-implementer skill with constitutional constraints
  - **Explicitly instruct**: "Follow `.claude/output-styles/lesson.md` CoLearning standards (lines 302-435)"
  - Build Three-Role Partnership from start (not added later)
  - Natural CoLearning element integration (correct count, correct format)
  - Conversational, exploration-focused tone throughout
- **Validate**: Run validation-auditor for constitutional alignment

**Time**: 30-45 minutes per lesson

**Example scenario**:
- Lesson with forward references (uses concepts before introduction)
- Tool-driven language ("Use AI to do X")
- Poor narrative flow
- **Decision**: Fresh start with spec/plan as source of truth, constitutional grounding, output style formatting

##### Option 4: No Change (Already embodies constitutional values)

**When to choose**:
- ✅ Constitutional alignment is **excellent**
- ✅ Content quality is **excellent**
- ✅ All requirements met **authentically** (not just mechanically)
- ✅ CoLearning element count matches proficiency tier
- ✅ CoLearning element formats match output style exactly
- ✅ Nothing to improve

**What this involves**:
- Validate with validation-auditor (quick check)
- Document as exemplar (can reference for other lessons)
- Move to next lesson

**Time**: 2-3 minutes (validation only)

---

#### Partnership Decision Points

**Always consult user before**:
- Full regeneration (existing content will be replaced)
- Spec/plan modifications
- Significant structural changes to excellent content

**Present options clearly**:

```markdown
**Lesson X Recommendation**: Surgical Edit (Remove Duplication)

**Assessment**:
- Constitutional Alignment: Good (has CoLearning elements, correct formats mostly)
- Content Quality: Excellent (clear writing, effective examples)
- **Issue**: 5 CoLearning elements (2💬 2🎓 1🤝); expected 1 of each type (3 total); 1 💬 is generic

**Rationale**:
Content is excellent and preservable. Issue is duplication (2 of some types) and one generic element. Surgical removal of duplicates/generic content preserves quality while achieving consistent 1+1+1=3 pattern.

**Options**:
1. **Surgical Edit** (recommended): Remove 1 generic 💬 and 1 duplicate 🎓, keep best of each type, standardize formats (10 min)
2. **Enhanced Regen**: Regenerate with preserved examples (thorough but unnecessary given excellent quality)
3. **No Change**: Accept inconsistent pattern (not recommended)

**Your preference?**
```

---

## III. EXECUTION WITH CONSTITUTIONAL INTEGRITY

### Surgical Edit: Contextual Insertion/Removal/Standardization

**Principle**: Modify constitutional elements **naturally**, not mechanically. **Quality over quantity.**

#### Process

**1. Identify Natural Insertion/Removal Points**

**For Adding Elements** (when below target count):
- After concepts are introduced (natural pause point)
- Before section transitions (bridges to next topic)
- After examples (opportunity for reflection)
- Before "Try With AI" section (final reinforcement)

**For Removing Elements** (when above target count or low quality):
- Generic elements (could apply to any lesson, not specific to this content)
- Redundant elements (repeat same concept/pattern as another element)
- Poorly placed elements (interrupt flow, break momentum)
- Low-quality elements (don't encourage exploration, restate content, passive copying)

**Avoid**:
- Mid-paragraph insertions (breaks flow)
- Interrupting narrative momentum
- Forcing elements where they don't fit naturally

**2. Generate/Standardize/Remove CoLearning Elements**

**CRITICAL**: Before generating or editing CoLearning elements, **RE-READ `.claude/output-styles/lesson.md` lines 302-435** to ensure 100% format consistency.

**Each element should be**:
- **Specific** to lesson content (not generic)
- **Constitutional** in spirit (demonstrates Three-Role Partnership)
- **Natural** in placement (enhances, doesn't interrupt)
- **Conversational** in tone (not preachy or mechanical)
- **Correctly formatted** (exact match to output style)

**Format Standardization Examples**:

**❌ INCORRECT (before)**:
```markdown
#### 💬 AI Colearning Prompt

> **Explore with your AI**: "Why does Python use this pattern?"
```

**✅ CORRECT (after)**:
```markdown
#### 💬 AI Colearning Prompt
> "Why does Python use this pattern instead of [alternative]?"
```

**❌ INCORRECT (before)**:
```markdown
#### 🤝 Practice Exercise

**Quick Test**: Ask your AI: "Create a function that calculates X."
```

**✅ CORRECT (after)**:
```markdown
#### 🤝 Practice Exercise

> **Ask your AI**: "Create a function that calculates X. Then explain [conceptual aspect] step-by-step."

**Expected Outcome**: [What student should understand after AI response]
```

**Quality Check Examples**:

**❌ GENERIC (remove)**:
```markdown
#### 💬 AI Colearning Prompt
> "Ask your AI to explain how variables work."
```
*Rationale*: Generic (not specific to this lesson's content); could apply to any variables lesson.

**✅ SPECIFIC (keep)**:
```markdown
#### 💬 AI Colearning Prompt
> "Why does Python use dynamic typing for variables instead of requiring explicit type declarations like Java or C++?"
```
*Rationale*: Specific to Python's design philosophy; encourages exploration of language tradeoffs.

**❌ LOW QUALITY (remove)**:
```markdown
#### 🎓 Expert Insight
> Functions are important in programming.
```
*Rationale*: Restates obvious fact; provides no strategic depth or pedagogical insight.

**✅ HIGH QUALITY (keep)**:
```markdown
#### 🎓 Expert Insight
> In AI-native development, you don't memorize 50 string methods—you understand text transformation intent. When you need case conversion, you specify "make this lowercase" and AI handles the syntax. Your job: recognize when text transformation solves your problem.
```
*Rationale*: Reframes learning from memorization to understanding; demonstrates "syntax is cheap, semantics is gold" mantra; specific, actionable insight.

**3. Insert/Remove/Standardize via Edit Tool**

```markdown
For adding element:
  old_string: [exact text from insertion point]
  new_string: [exact text] + [generated CoLearning element in output style format]

For removing element:
  old_string: [entire CoLearning element including heading and content]
  new_string: [empty string - complete removal]

For standardizing format:
  old_string: [existing CoLearning element with incorrect format]
  new_string: [same content, corrected to output style format]
```

**4. Remove Post-Sections (if any)**

**Common violations**:
- "## What's Next" after "Try With AI"
- "## Key Takeaways" after "Try With AI"
- "## Summary" after "Try With AI"
- "## Completion Checklist" after "Try With AI"

```markdown
For each post-section:
  old_string: [entire section including heading]
  new_string: [empty string - complete removal]
```

**5. Validate Natural Flow and Quality**

**After modifications, read lesson**:
- Do elements feel natural or forced?
- Does flow remain smooth?
- Is tone consistent (conversational, not preachy)?
- **Is total element count appropriate for proficiency tier?** (not over-stuffed)
- **Do all elements use output style format exactly?** (no drift)
- **Are all elements specific to lesson content?** (not generic)

**If changes feel forced**: Adjust placement or wording.
**If still over-stuffed**: Remove additional low-quality elements.
**If formats don't match output style**: Re-read output style and correct.

---

### Enhanced Regeneration: Preserve + Improve

**Principle**: Keep what embodies constitutional values, regenerate what doesn't.

#### Process

**1. Extract Quality Content**

**Read lesson thoroughly**, identify:
- **Excellent examples** (code samples, analogies, real-world scenarios)
- **Effective explanations** (clear, accessible, engaging)
- **Good analogies** (help understanding)
- **Existing constitutional elements** (if any Three-Role Partnership demonstrations)
- **High-quality CoLearning elements** (if any meet output style standards)

**Document preserved content**:
```markdown
## Content to Preserve (Lesson X)

**Example 1** (lines 89-105):
[code sample or explanation]
Rationale: Clear, effective teaching; already demonstrates exploration

**Analogy** (line 234):
"Think of it like..."
Rationale: Helps understanding; relatable

**Existing 💬 Prompt** (line 156):
> "Why does Python use X instead of Y?"
Rationale: Excellent format, specific to content, encourages exploration
```

**2. Identify Regeneration Targets**

**Sections needing rewrite**:
- Documentation tone areas ("Functions are defined as...")
- Missing CoLearning integration
- Tool-driven language ("Use AI to do X" vs. "Collaborate with AI to explore...")
- Forward references or ordering issues
- Over-stuffed CoLearning sections (too many elements)
- Format drift (CoLearning elements not matching output style)

**3. Regenerate with Constitution + Output Style + Preservation**

**Use content-implementer skill** with specific instructions:

```markdown
Invoke content-implementer:
- Spec: [path to spec.md, if exists]
- Plan: [path to plan.md, lesson N section, if exists]
- Preserved Content: [extracted examples and explanations from Step 1]
- Constitutional Constraints:
  - Emphasize Three-Role AI Partnership throughout
  - Conversational, exploration-focused tone
  - "Specs Are the New Syntax" framing for code examples
  - No forward references
  - Cognitive load management (A1: 5 max, A2: 7 max, B1: 10 max concepts)
- **CoLearning Element Standards** (CRITICAL):
  - **MUST READ `.claude/output-styles/lesson.md` lines 302-435 before generating**
  - Element pattern: **1 of each type** (💬 + 🎓 + 🤝 = 3 total)
  - Use exact formats from output style (💬🎓🤝)
  - Ensure elements are specific to lesson content (not generic)
  - Consistency: Same pattern (1+1+1=3) across all lessons
- Output: [lesson file path]
```

**4. Validate Hybrid Result**

**Check**:
- ✅ Does it preserve quality? (examples, explanations intact)
- ✅ Does it embody constitution? (Three-Role Partnership, co-learning)
- ✅ Does it flow naturally? (not disjointed)
- ✅ Is tone consistent? (conversational throughout)
- ✅ **CoLearning element count matches proficiency tier?** (from output style)
- ✅ **CoLearning element formats match output style exactly?** (no drift)
- ✅ **CoLearning elements specific to content?** (not generic)

**Run validation-auditor** for constitutional alignment validation.

---

### Full Regeneration: Constitutional Grounding

**Principle**: Generate from constitutional intent, using spec/plan as source of truth, output style as formatting standard.

#### Process

**1. Review Spec/Plan (if they exist)**

**Read spec.md**:
- What are learning objectives?
- What proficiency level (A1/A2/B1/B2/C1)?
- What's the scope (awareness vs. mastery)?

**Read plan.md** (specific lesson section):
- What's the lesson structure?
- What concepts should be covered?
- What's the intended flow?

**If no spec/plan**: Use existing lesson objectives (from frontmatter metadata).

**2. Ground in Constitution AND Output Style**

**Ask** (constitutional grounding):
- Which of the 18 principles apply to this content type?
- How should Three-Role Partnership manifest here specifically?
- What does "Specs Are the New Syntax" mean in this context?
- Which Graduated Teaching tier is appropriate (Book teaches / AI Companion / AI Orchestration)?

**Ask** (output style grounding):
- What proficiency tier is this lesson? (A1-A2 / A2-B1 / B1-B2 / B2-C1)
- What's the CoLearning element pattern? (1 of each type: 💬 + 🎓 + 🤝 = 3 total)
- What are the exact CoLearning element formats? (💬🎓🤝 from output style)

**3. Generate with Constitutional Lens + Output Style Standards**

**Use content-implementer skill** with constitutional constraints:

```markdown
Invoke content-implementer:
- Spec: [spec.md path]
- Plan: [plan.md lesson section]
- Constitutional Framework:
  - Build Three-Role AI Partnership from start (demonstrate in narrative, not just add elements)
  - Conversational, exploration-focused tone throughout
  - "Specs Are the New Syntax" emphasis (for code lessons)
  - No forward references (pedagogical ordering strict)
  - Cognitive load management (A1: 5 max, A2: 7 max, B1: 10 max concepts)
- **CoLearning Element Standards** (CRITICAL):
  - **MUST READ `.claude/output-styles/lesson.md` lines 302-435 FIRST**
  - Proficiency tier: [A1-A2 / A2-B1 / B1-B2 / B2-C1]
  - Element pattern: **1 of each type** (💬 + 🎓 + 🤝 = 3 total)
  - Use exact formats from output style (💬🎓🤝)
  - Natural integration throughout lesson (not mechanical insertion)
  - Specific to lesson content (not generic)
  - Consistency: Same pattern (1+1+1=3) across all lessons
- Output: [lesson file path]
```

**4. Validate Against Constitution + Output Style**

**Run validation-auditor**:
```bash
Invoke validation-auditor:
- File: [lesson file path]
- Focus: Constitutional alignment (Three-Role Partnership, CoLearning elements, tone, ordering)
- CoLearning validation: Count matches tier, formats match output style, quality is excellent
```

**Check embodiment** (not just presence):
- Does narrative demonstrate partnership authentically?
- Do CoLearning elements encourage exploration?
- Is tone conversational throughout?
- Does it teach specs-first for code examples?
- **Are CoLearning element quantity and format correct?**

---

## IV. VALIDATION & PARTNERSHIP REPORTING

### Chapter-Level Constitutional Coherence

**After all lessons processed**, validate chapter as integrated whole.

#### Cross-Lesson Consistency

**Check**:
- ✅ Does chapter demonstrate **Three-Role Partnership progression**? (not just present in isolated lessons)
- ✅ Are CoLearning elements **consistent** across lessons? (1 of each type per lesson: 💬🎓🤝 = 3 total)
- ✅ Are CoLearning element **formats consistent** across lessons? (all match output style exactly)
- ✅ Does tone remain **conversational** throughout chapter?
- ✅ Is **Graduated Teaching Pattern** evident across lessons? (if applicable)
- ✅ No **forward references** across lessons? (Lesson N only uses concepts from 1 to N-1)
- ✅ **Terminology consistent** across lessons?

**If issues found**:
- Identify problematic lessons
- May need additional surgical edits for consistency
- Consult user on approach

#### Constitutional Embodiment

**Beyond structural compliance**, assess:
- Does chapter **teach with AI** (not just about AI)?
- Does it emphasize **specs over syntax** (if code-heavy)?
- Does it **validate alongside generation**?
- Does it demonstrate **co-learning partnership** authentically?

**Output**:

```markdown
## Chapter-Level Validation

**Spec/Plan Consistency**: ✅ PASS
  ✓ All spec objectives covered in lessons
  ✓ All plan concepts present
  ✓ CEFR progression maintained

**Cross-Lesson Consistency**: ✅ PASS
  ✓ No forward references across lessons
  ✓ Prerequisite chain intact (L1→L2→L3...)
  ✓ Terminology consistent
  ✓ CoLearning elements consistent (1 of each type per lesson, matching formats)

**Constitutional Embodiment**: ✅ PASS
  ✓ All 18 principles verified at chapter level
  ✓ Three-Role Partnership demonstrated progressively
  ✓ Co-learning emphasized throughout
  ✓ "Specs Are the New Syntax" framing present (for code chapters)

**Final Verdict**: ✅ PASS (Chapter ready for publication)
```

---

### Partnership Report to User

**Use template**: `.specify/templates/constitution-sync-report-template.md`

**Key Sections**:

```markdown
# ✅ CONSTITUTION SYNC COMPLETE: Chapter [N]

## Executive Summary

**Constitutional Alignment**: [Before X% → After Y%]
**Approach**: [Interventions used and rationale]
**Content Quality**: [Preserved / Improved]
**Time**: [Total time invested]

## What Changed and Why

**Constitutional Gaps Addressed**:
- [Not just "added elements" but "integrated Three-Role Partnership demonstration"]
- [Specific constitutional principles that were missing and how they're now embodied]
- [CoLearning element count adjustments (removed over-stuffing, standardized formats)]

**Quality Improvements** (beyond compliance):
- [How content got better, not just compliant]
- [Examples preserved, tone improved, flow enhanced]
- [Professional density achieved (quality over quantity)]

## Per-Lesson Decisions

| Lesson | Alignment Before | Intervention | Rationale | Result |
|--------|-----------------|--------------|-----------|---------|
| L1 | Good (70%) | Surgical Edit (Remove) | Excellent content, 5 elements (2💬 2🎓 1🤝), 2 generic | Removed generic elements, kept 1 of each type (3 total), standardized formats |
| L2 | Needs Work (55%) | Enhanced Regen | Good examples, documentation tone, format drift | Preserved examples, regenerated narrative with 1+1+1=3 pattern |
| L3 | Excellent (95%) | No Change | Already has 1+1+1=3 pattern, correct formats | Validation only |

## Why This Approach Was Optimal

**vs. All Surgical Edit**:
- Would have missed Lesson 2's tone/partnership issues
- Quality: Lower (can't fix deep problems with insertions alone)

**vs. All Full Regeneration**:
- Would have replaced Lesson 1's excellent narrative unnecessarily
- Quality: Risk losing existing excellence
- Time: 2-3 hours (vs. actual 45 minutes)

**Intelligent Hybrid**:
- Each lesson got what it needed
- Quality: Maximized (preserve good, fix bad)
- Time: Optimal (45 minutes for 3 lessons)
- Professional density: Achieved (no over-stuffing)

## Constitutional Compliance Achieved

✅ All 18 constitutional principles verified compliant
✅ CoLearning elements (1 of each type per lesson: 💬🎓🤝 = 3 total, quality-checked)
✅ CoLearning element formats (100% match to output style, no drift)
✅ Lesson closure pattern (100% compliant)
✅ Pedagogical ordering (no forward references)
✅ Three-Role Partnership (demonstrated authentically throughout)
✅ "Specs Are the New Syntax" (emphasized for code lessons)

## Recommendations

**Chapter is publication-ready.**

**Next Steps**:
1. **Review changes**: `git diff` to see specific edits
2. **Test build**: Ensure Docusaurus builds without errors
3. **Commit changes**:
   ```bash
   git add book-source/docs/[chapter-path]/*.md
   git commit -m "Constitution sync: Chapter [N] aligned with v3.1.3

   - Integrated Three-Role AI Partnership throughout
   - Optimized CoLearning elements (removed over-stuffing, standardized formats)
   - Fixed lesson closure violations
   - Preserved excellent narrative quality
   - Achieved professional density (quality over quantity)"
   ```
4. **Process next chapter?**: Run `/sp.constitution-sync [N+1]`

**Your decision?**
```

---

## V. CONTINUOUS IMPROVEMENT

### Learning from Each Sync

**After completing sync**, reflect:

**What worked well?**
- Which interventions preserved quality most effectively?
- Which CoLearning insertions/removals felt most natural?
- What constitutional elements resonated most in this chapter?
- Did professional density improve? (no over-stuffing)

**What to improve next time?**
- Were any interventions too heavy-handed?
- Did any insertions feel forced?
- How can next sync be more constitutionally grounded?
- Were format standards followed consistently?

**Document insights** for future syncs.

---

### Evolving Constitutional Understanding

**As constitution evolves**, update your understanding:

**When new principles added**:
- How do they manifest in content?
- Which content types do they apply to?
- What does authentic embodiment look like?

**When existing principles clarified**:
- Adjust assessment lens
- Re-evaluate what "good" looks like

**When output style updates**:
- Re-read output style for new standards
- Update format examples in this command if needed
- Ensure all future syncs use updated standards

**When exemplars identified**:
- Study what makes them excellent
- Apply patterns to future syncs

**Stay grounded in constitution as living document, not static rulebook.**

---

## Success Criteria

✅ **Constitutional Embodiment** (not just mechanical compliance)
✅ **Content Quality** (preserved or improved, never degraded)
✅ **User Partnership** (collaborative decisions, not dictation)
✅ **Publication Readiness** (chapter fully aligned and ready to ship)
✅ **Pedagogical Integrity** (teaches values, not just topics)
✅ **Professional Density** (quality over quantity, no over-stuffing)
✅ **Format Consistency** (100% match to output style standards)

---

## Philosophy Reminder

**Constitution-sync is not about**:
- ❌ Checking boxes mechanically
- ❌ Following formulas blindly
- ❌ Compliance for compliance's sake
- ❌ Adding maximum CoLearning elements ("more is better")
- ❌ Using format variations ("close enough")

**Constitution-sync is about**:
- ✅ Embodying constitutional values authentically
- ✅ Preserving and enhancing content quality
- ✅ Partnering with user intelligently
- ✅ Teaching with AI, not just about AI
- ✅ **Quality over quantity** (2-3 excellent elements > 6-8 mediocre ones)
- ✅ **Professional density** (respect learner time and attention)
- ✅ **Format precision** (exact match to output style, no drift)

**Every decision should ask**: *Does this help content better embody what the constitution stands for while respecting professional learners' needs?*

---

**This command brings content into authentic alignment with constitutional values through intelligent, judgment-driven partnership, prioritizing quality over quantity and format consistency over variation.**
