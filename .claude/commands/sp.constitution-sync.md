# /sp.constitution-sync: Constitutional Alignment Through Intelligent Partnership

**Version**: 2.0.0 (Restructured: Philosophy-First)
**Created**: 2025-01-10
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
- **Principle 3**: Specification-First Development
- **Principle 2**: AI as Co-Learning Partner
- **Principle 5**: Validation-Before-Trust

**8 Core Philosophies**:
- **Co-Learning**: Bidirectional learning (human ↔ AI refine each other)
- **Evals-First**: Define success criteria before specs
- **Spec-Driven**: Specification is the new syntax
- **Validation-First**: Never trust AI output without verification

**Nine Pillars**: AI CLI, Markdown, MCP, AI-First IDEs, Cross-Platform, TDD, SDD, Composable Skills, Cloud-Native

**Output Style Requirements**:
- 💬 **AI Colearning Prompts** (1-4 per lesson): Exploration-focused questions
- 🎓 **Expert Insights** (2-4 per lesson): Strategic depth and pedagogical insights
- 🤝 **Practice Exercises** (1-4 per lesson): Hands-on collaborative practice
- **Lesson Closure**: "Try With AI" is final section (no post-sections)

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

#### Step 1.1: Read Constitution Deeply

```bash
Read .specify/memory/constitution.md
```

**Extract Understanding** (not just data):
- **Version**: What changed in latest version? (check HTML comment at top)
- **Sync Impact**: What's the impact of recent changes? (from Sync Impact Report)
- **Applicable Principles**: Which of the 18 principles apply to this chapter type?
- **Target Level**: What's the proficiency level? (A1-beginner, A2-elementary, B1-intermediate)
- **Constitutional Intent**: Why do these rules exist? (understand spirit, not just letter)

#### Step 1.2: Categorize by Impact and Context

**High-Impact Requirements** (must be present in ALL lessons):
- CoLearning elements (💬🎓🤝) demonstrating Three-Role Partnership
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
**Target Level**: [A1-Beginner / A2-Elementary / B1-Intermediate]
**Primary Principles**: [List 3-5 most relevant of the 18 principles]

**What "Good" Looks Like**:
- [Describe ideal embodiment of constitutional values for THIS chapter]
- [Not generic—specific to chapter content and audience]

**Common Gaps to Watch For**:
- [Based on chapter type, predict likely violations]
- [Example: Conceptual chapters often missing hands-on challenges]
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

**Presence**:
- Are 💬 AI Colearning Prompts present? (Expected: 1-4 per lesson)
- Are 🎓 Expert Insights present? (Expected: 2-4 per lesson)
- Are 🤝 Practice Exercises present? (Expected: 1-4 per lesson)

**Quality** (more important than quantity):
- Do 💬 prompts encourage **exploration** ("What happens if...") or just "Ask AI to write X"?
- Do 🎓 insights provide **strategic depth** and pedagogical perspective (why this matters, non-obvious implications) or just restate content?
- Do 🤝 exercises practice **collaborative learning** (iteration, validation, partnership) or passive copying?

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
- [Specific examples: "Instructor Commentary on line 145 demonstrates Three-Role Partnership clearly"]

**Gaps**:
- [What's missing or misaligned?]
- [Specific, not generic: "Prompts present but don't encourage exploration—they're task-focused ('Ask AI to write X')"]

**Content Quality**: [Excellent / Good / Needs Improvement]
- Writing clarity: [assessment]
- Examples effectiveness: [assessment]
- Narrative flow: [assessment]

**Recommended Intervention**: [Surgical Edit / Enhanced Regeneration / Full Regeneration / No Change]

**Rationale**: [Why this intervention preserves quality while achieving alignment]
- [Example: "Surgical edit preserves excellent narrative and examples, adds 4 targeted CoLearning elements at natural break points"]
```

---

### Phase 3: Intelligent Intervention Decisions

**Your Role**: AI as **Co-Worker** — Partner with user to decide best approach for each lesson.

#### Decision Framework: Judgment, Not Formula

##### Option 1: Surgical Edit (Preserve excellent content, add missing elements)

**When to choose**:
- ✅ Content quality is **excellent** (clear writing, effective teaching, good examples)
- ✅ Gaps are **structural** (missing CoLearning elements, post-sections to remove)
- ✅ Tone and partnership language already appropriate
- ✅ Can insert elements naturally without disrupting flow
- ✅ No pedagogical ordering violations

**What this involves**:
- Insert 💬🎓🤝 elements at natural break points (after concepts introduced, before transitions)
- Remove post-sections (e.g., "What's Next" after "Try With AI")
- Enhance existing content with partnership language (minimal rewording)
- Validate: Ensure insertions feel natural, not forced

**Time**: 10-15 minutes per lesson

**Example scenario**:
- Lesson with excellent narrative and examples
- Missing CoLearning elements (0/3 types present)
- Good conversational tone already
- **Decision**: Add 3-4 targeted insertions (💬🎓🤝) at natural points

##### Option 2: Enhanced Regeneration (Preserve good parts, regenerate problem areas)

**When to choose**:
- ✅ Content quality is **good** (worth preserving examples, explanations, analogies)
- ✅ Gaps are **mixed** (structural + tone/ordering/partnership issues)
- ✅ Core content solid but needs constitutional framing
- ✅ Can extract and reuse quality content with new narrative

**What this involves**:
- **Extract** excellent examples, explanations, analogies, code samples
- **Identify** sections needing rewrite (documentation tone, missing partnership, ordering issues)
- **Regenerate** with constitution + preserved content:
  - Use lesson-writer skill
  - Provide extracted content as "preserve these examples"
  - Emphasize constitutional framing (Three-Role Partnership, exploration focus)
  - Add CoLearning elements throughout (not just inserted afterward)
- **Validate**: Does it preserve quality? Does it embody constitution? Natural flow?

**Time**: 20-30 minutes per lesson

**Example scenario**:
- Lesson with solid code examples (preserve)
- Documentation tone throughout (regenerate narrative)
- Missing Three-Role Partnership framework (add via regeneration)
- **Decision**: Extract examples, regenerate narrative with constitutional framing

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
- **Ground in constitution**:
  - Which principles apply to this content?
  - How should Three-Role Partnership manifest here specifically?
  - What does "Specs Are the New Syntax" mean in this context?
- **Generate** with constitutional lens:
  - Use lesson-writer skill with constitutional constraints
  - Build Three-Role Partnership from start (not added later)
  - Natural CoLearning element integration
  - Conversational, exploration-focused tone throughout
- **Validate**: Run technical-reviewer for constitutional alignment

**Time**: 30-45 minutes per lesson

**Example scenario**:
- Lesson with forward references (uses concepts before introduction)
- Tool-driven language ("Use AI to do X")
- Poor narrative flow
- **Decision**: Fresh start with spec/plan as source of truth, constitutional grounding

##### Option 4: No Change (Already embodies constitutional values)

**When to choose**:
- ✅ Constitutional alignment is **excellent**
- ✅ Content quality is **excellent**
- ✅ All requirements met **authentically** (not just mechanically)
- ✅ Nothing to improve

**What this involves**:
- Validate with technical-reviewer (quick check)
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
**Lesson X Recommendation**: Enhanced Regeneration

**Assessment**:
- Constitutional Alignment: Needs Work (missing Three-Role Partnership, documentation tone)
- Content Quality: Good (excellent code examples, clear explanations)

**Rationale**:
Content has excellent examples worth preserving, but narrative needs constitutional framing. Enhanced regen preserves quality while achieving alignment.

**Options**:
1. **Enhanced Regen** (recommended): Preserve examples, regenerate narrative with constitutional framing (20-30 min)
2. **Surgical Edit**: Add elements only (faster but won't fix tone/partnership issues)
3. **Full Regen**: Fresh start (thorough but loses existing quality examples)

**Your preference?**
```

---

## III. EXECUTION WITH CONSTITUTIONAL INTEGRITY

### Surgical Edit: Contextual Insertion

**Principle**: Add constitutional elements **naturally**, not mechanically.

#### Process

**1. Identify Natural Insertion Points**

Look for:
- After concepts are introduced (natural pause point)
- Before section transitions (bridges to next topic)
- After examples (opportunity for reflection)
- Before "Try With AI" section (final reinforcement)

**Avoid**:
- Mid-paragraph insertions (breaks flow)
- Interrupting narrative momentum
- Forcing elements where they don't fit naturally

**2. Generate Contextual CoLearning Elements**

**Each element should be**:
- **Specific** to lesson content (not generic)
- **Constitutional** in spirit (demonstrates Three-Role Partnership)
- **Natural** in placement (enhances, doesn't interrupt)
- **Conversational** in tone (not preachy or mechanical)

**Writing Style Guidelines** (apply to ALL CoLearning elements and content):

**Punctuation & Emphasis** (apply judiciously with context-awareness):
- **Em-dash (—)**: ONLY fix when it creates grammatical issues or hinders readability
  - ✅ KEEP: "The developer's role is changing—and it's happening faster than expected." (dramatic break, grammatically sound)
  - ✅ KEEP: "Understanding intent (not memorizing syntax) is the new skill." (parenthetical aside)
  - ❌ FIX: "The job hasn't expanded arbitrarily—the technology landscape has integrated..." when it connects two independent clauses that should be separate sentences
  - **Judgment required**: If removing the em-dash creates awkward flow or changes meaning, KEEP it. Only fix actual violations.

- **Bold formatting**: ONLY fix when it's clearly decorative or excessive
  - ✅ KEEP: "A **variable** stores data..." (new technical term on first use)
  - ✅ KEEP: "The **primary skill** is specification-writing..." (critical concept requiring emphasis)
  - ✅ KEEP: "**Path 1 (Fine-Tuning)**" (structural heading or important distinction)
  - ✅ KEEP: "**there are two paths**" when emphasizing a key structural point
  - ❌ FIX ONLY: Multiple bold words in a single prompt where emphasis is decorative (e.g., "Give me **realistic** scenarios for **my** situation")
  - **Judgment required**: If bold serves pedagogical purpose (emphasis on key distinction, first use of term, structural clarity), KEEP it.

- **ALL CAPS**: ONLY fix when used for emphasis (replace with italic or normal)
  - ❌ FIX: "Give me REALISTIC scenarios for MY situation..." → "Give me realistic scenarios for my situation..."
  - ✅ KEEP: ALL CAPS in acronyms (API, LMS, MCP) or proper names
  - **Judgment required**: Distinguish between emphasis (fix) and legitimate uses (keep).

- **Natural flow**: ONLY intervene when punctuation genuinely disrupts coherence
  - FIX sentence structure ONLY when:
    - Em-dash connects independent clauses that should be periods
    - Sentence is grammatically incorrect after em-dash removal
    - Multiple em-dashes create choppy reading
  - PRESERVE existing flow when:
    - Em-dash serves dramatic or stylistic purpose effectively
    - Sentence is grammatically sound
    - Readability is not hindered

**CRITICAL**: Apply these guidelines with **judgment and restraint**. Only fix actual violations that harm clarity, grammar, or readability. Do NOT mechanically remove all em-dashes, bold formatting, or caps without considering context and pedagogical intent.

**Tone**:
- Natural conversational language ("Help me understand" not "Ask your AI to explain")
- Professional and respectful (no condescension, no gatekeeping terms)
- Direct and clear (avoid overly dramatic or breathless phrasing)

**Examples**:

**💬 AI Colearning Prompt** (after introducing developer role evolution):
```markdown
#### 💬 AI Colearning Prompt

> **Explore with your AI**: "The lesson mentions developers shifting from 'typist' to 'orchestrator.' Help me understand this transition using a concrete analogy from another profession that went through similar transformation."
```

**Quality check**:
- ✅ Encourages exploration (not just "ask AI to explain")
- ✅ Demonstrates AI as Teacher (student learns from AI)
- ✅ Specific to lesson content (developer role evolution)
- ✅ Conversational tone (natural, not command-driven)

**Style Guidelines**:
- Use natural conversational language ("Help me understand" not "Ask your AI")
- Avoid ALL CAPS for emphasis (use italic *emphasis* or bold **emphasis** sparingly)
- Use em-dash (—) only for parenthetical breaks, not to join phrases
- Placeholders in square brackets should be lowercase: [describe your situation]
- Keep prompts feeling like natural dialogue with AI partner

**🎓 Expert Insight** (after explaining Three-Role Partnership):
```markdown
#### 🎓 Expert Insight

> Notice how this partnership is bidirectional. You specify what you need (AI learns your intent), AI suggests patterns you don't know yet (you learn from AI), and together you create what neither could alone. This isn't "using a tool"; it's co-creation.
```

**Quality check**:
- ✅ Provides strategic depth and pedagogical insight (why this matters)
- ✅ Demonstrates constitutional principle (co-learning)
- ✅ Enhances understanding (not just repeating content)
- ✅ Natural placement (after concept introduction)

**🤝 Practice Exercise** (practicing specification-writing):
```markdown
#### 🤝 Practice Exercise

**Quick Test**: Ask your AI: "I want to build a simple to-do list app. What information do you need from me to implement it well?"

**What you're practicing**: Specification-writing. The AI will ask about features, data structure, UI preferences, showing you what good specs include.
```

**Quality check**:
- ✅ Hands-on collaborative practice (not passive reading)
- ✅ Demonstrates AI as Student (adapts to your specs)
- ✅ Teaches "Specs Are the New Syntax" principle
- ✅ Low barrier to entry (quick test)

**3. Insert via Edit Tool**

```markdown
For each insertion:
  old_string: [exact text from insertion point]
  new_string: [exact text] + [generated CoLearning element]
```

**4. Remove Post-Sections (if any)**

**Common violations**:
- "## What's Next" after "Try With AI"
- "## Key Takeaways" after "Try With AI"
- "## Summary" after "Try With AI"

```markdown
For each post-section:
  old_string: [entire section including heading]
  new_string: [empty string - complete removal]
```

**5. Validate Natural Flow**

**After insertions, read lesson**:
- Do elements feel natural or forced?
- Does flow remain smooth?
- Is tone consistent (conversational, not preachy)?

**If insertions feel forced**: Adjust placement or wording.

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

**Document preserved content**:
```markdown
## Content to Preserve (Lesson X)

**Example 1** (lines 89-105):
[code sample or explanation]
Rationale: Clear, effective teaching; already demonstrates exploration

**Analogy** (line 234):
"Think of it like..."
Rationale: Helps understanding; relatable
```

**2. Identify Regeneration Targets**

**Sections needing rewrite**:
- Documentation tone areas ("Functions are defined as...")
- Missing CoLearning integration
- Tool-driven language ("Use AI to do X" vs. "Collaborate with AI to explore...")
- Forward references or ordering issues

**3. Regenerate with Constitution + Preservation**

**Use lesson-writer skill** with specific instructions:

```markdown
Invoke lesson-writer:
- Spec: [path to spec.md, if exists]
- Plan: [path to plan.md, lesson N section, if exists]
- Preserved Content: [extracted examples and explanations from Step 1]
- Constitutional Constraints:
  - Emphasize Three-Role AI Partnership throughout
  - Conversational, exploration-focused tone
  - Add CoLearning elements naturally (💬🎓🤝)
  - "Specs Are the New Syntax" framing for code examples
  - No forward references
- Output: [lesson file path]
```

**4. Validate Hybrid Result**

**Check**:
- ✅ Does it preserve quality? (examples, explanations intact)
- ✅ Does it embody constitution? (Three-Role Partnership, co-learning)
- ✅ Does it flow naturally? (not disjointed)
- ✅ Is tone consistent? (conversational throughout)

**Run technical-reviewer** for constitutional alignment validation.

---

### Full Regeneration: Constitutional Grounding

**Principle**: Generate from constitutional intent, using spec/plan as source of truth.

#### Process

**1. Review Spec/Plan (if they exist)**

**Read spec.md**:
- What are learning objectives?
- What proficiency level (A1/A2/B1)?
- What's the scope (awareness vs. mastery)?

**Read plan.md** (specific lesson section):
- What's the lesson structure?
- What concepts should be covered?
- What's the intended flow?

**If no spec/plan**: Use existing lesson objectives (from frontmatter metadata).

**2. Ground in Constitution**

**Ask**:
- Which of the 18 principles apply to this content type?
- How should Three-Role Partnership manifest here specifically?
- What does "Specs Are the New Syntax" mean in this context?
- Which Graduated Teaching tier is appropriate (Book teaches / AI Companion / AI Orchestration)?

**3. Generate with Constitutional Lens**

**Use lesson-writer skill** with constitutional constraints:

```markdown
Invoke lesson-writer:
- Spec: [spec.md path]
- Plan: [plan.md lesson section]
- Constitutional Framework:
  - Build Three-Role AI Partnership from start (demonstrate in narrative, not just add elements)
  - Natural CoLearning element integration (💬🎓🤝✨ not mechanical insertion)
  - Conversational, exploration-focused tone throughout
  - "Specs Are the New Syntax" emphasis (for code lessons)
  - No forward references (pedagogical ordering strict)
  - Cognitive load management (A1: 5 max, A2: 7 max, B1: 10 max concepts)
- Output: [lesson file path]
```

**4. Validate Against Constitution**

**Run technical-reviewer**:
```bash
Invoke technical-reviewer:
- File: [lesson file path]
- Focus: Constitutional alignment (Three-Role Partnership, CoLearning elements, tone, ordering)
```

**Check embodiment** (not just presence):
- Does narrative demonstrate partnership authentically?
- Do CoLearning elements encourage exploration?
- Is tone conversational throughout?
- Does it teach specs-first for code examples?

---

## IV. VALIDATION & PARTNERSHIP REPORTING

### Chapter-Level Constitutional Coherence

**After all lessons processed**, validate chapter as integrated whole.

#### Cross-Lesson Consistency

**Check**:
- ✅ Does chapter demonstrate **Three-Role Partnership progression**? (not just present in isolated lessons)
- ✅ Are CoLearning elements **balanced** across lessons? (not heavy in L1, missing in L5)
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
  ✓ CoLearning elements balanced

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

**Quality Improvements** (beyond compliance):
- [How content got better, not just compliant]
- [Examples preserved, tone improved, flow enhanced]

## Per-Lesson Decisions

| Lesson | Alignment Before | Intervention | Rationale | Result |
|--------|-----------------|--------------|-----------|---------|
| L1 | Good (70%) | Surgical Edit | Excellent content, missing elements | 4 targeted insertions, preserved narrative |
| L2 | Needs Work (55%) | Enhanced Regen | Good examples, documentation tone | Preserved examples, regenerated narrative with partnership framing |
| L3 | Excellent (95%) | No Change | Already embodies values | Validation only |

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

## Constitutional Compliance Achieved

✅ All 18 constitutional principles verified compliant
✅ CoLearning elements (💬🎓🤝✨ 100% coverage, quality-checked)
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
   git commit -m "Constitution sync: Chapter [N] aligned with v3.1.2

   - Integrated Three-Role AI Partnership throughout
   - Added CoLearning elements (💬🎓🤝) naturally
   - Fixed lesson closure violations
   - Preserved excellent narrative quality"
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
- Which CoLearning insertions felt most natural?
- What constitutional elements resonated most in this chapter?

**What to improve next time?**
- Were any interventions too heavy-handed?
- Did any insertions feel forced?
- How can next sync be more constitutionally grounded?

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

---

## Philosophy Reminder

**Constitution-sync is not about**:
- ❌ Checking boxes mechanically
- ❌ Following formulas blindly
- ❌ Compliance for compliance's sake

**Constitution-sync is about**:
- ✅ Embodying constitutional values authentically
- ✅ Preserving and enhancing content quality
- ✅ Partnering with user intelligently
- ✅ Teaching with AI, not just about AI

**Every decision should ask**: *Does this help content better embody what the constitution stands for?*

---

**This command brings content into authentic alignment with constitutional values through intelligent, judgment-driven partnership.**
