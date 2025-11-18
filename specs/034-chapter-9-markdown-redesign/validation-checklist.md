# Chapter 9 Validation Checklist

**Feature**: Chapter 9 Markdown Redesign - Specification Language Approach
**Created**: 2025-11-18
**Purpose**: Constitutional compliance and quality validation checklist

---

## Constitutional Compliance Checks

### Lesson Ending Protocol (Constitution Principle 7)

**REQUIRED**:
- ✅ Each lesson MUST end with "## Try With AI" as ONLY final section

**FORBIDDEN**:
- ❌ No "What's Next" sections after "Try With AI"
- ❌ No "Key Takeaways" sections after "Try With AI"
- ❌ No "Summary" sections after "Try With AI"
- ❌ No standalone "Safety Note" sections (safety notes embedded inside "Try With AI" only, max 1-2 sentences)

**Validation Method**:
```bash
# Check last section header in each lesson
tail -50 <lesson>.md | grep -E "^## " | tail -1
# Expected: "## Try With AI" for all 5 lessons
```

---

### No-Code Constraint (Spec Requirement FR-004, FR-005, FR-006)

**FORBIDDEN Programming Code Examples**:
- ❌ Python syntax (`def `, `import `, `class `)
- ❌ Bash syntax (`#!/bin/bash`, `|`, `&&`)
- ❌ JavaScript/TypeScript syntax (`function `, `const `, `let `, `var `)
- ❌ Any implementation language syntax

**REQUIRED Specification Examples**:
- ✅ Feature lists (WHAT system provides)
- ✅ Acceptance criteria (WHAT success looks like)
- ✅ Expected outputs (WHAT user sees)
- ✅ API endpoint descriptions (WHAT request/response format)
- ✅ System requirements (WHAT behavior is expected)

**Validation Method**:
```bash
# Search for forbidden code patterns
grep -rE '(def |function |import |const |#!/bin/bash|```python|```bash|```javascript)' \
  book-source/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/*.md

# Expected: ZERO matches in student-facing lesson text
```

---

### Internal Label Prohibition (Constitution Principle 7)

**FORBIDDEN Internal Scaffolding Labels**:
- ❌ "Stage 1" / "Stage 2" / "Stage 3" / "Stage 4" in student-facing text
- ❌ "Layer 1" / "Layer 2" / "Layer 3" / "Layer 4" in student-facing text
- ❌ "Three Roles Framework" as section header
- ❌ "Three Roles in Action" as section header
- ❌ "AI as Teacher/Student/Co-Worker" as explicit section headers

**REQUIRED Instead**:
- ✅ Natural narrative flow demonstrating Three Roles WITHOUT labels
- ✅ Stage information in frontmatter metadata only (not student-facing)
- ✅ Phrases like "AI suggested...", "You refined...", "Together you converged..." (narrative, not labels)

**Validation Method**:
```bash
# Search for internal labels in student text
grep -rE '(Stage [0-9]|Layer [0-9]|Three Roles (Framework|in Action))' \
  book-source/docs/03-Markdown-Prompt-Context-Engineering/09-markdown-language-of-ai/*.md

# Expected: ZERO matches (labels only in frontmatter)
```

---

## Cognitive Load Compliance (A2 Tier)

### Concept Count Limits (Constitution Principle 2)

**A2 Tier Limits**:
- Maximum 7 concepts per lesson (5-7 range with heavy scaffolding)
- Concepts must be documented in frontmatter metadata

**Expected Concept Counts**:
- Lesson 1: 6 concepts (foundation)
- Lesson 2: 7 concepts (lists + AI collaboration)
- Lesson 3: 7 concepts (code blocks + Three Roles)
- Lesson 4: 5 concepts (links/images/templates)
- Lesson 5: 8 concepts (capstone composition - exception allowed)

**Validation Method**:
```yaml
# Check frontmatter in each lesson
concepts: N  # Must match actual concept count in lesson body
```

---

### Scaffolding Requirements (Constitution Principle 2)

**REQUIRED for A2 Tier**:
- ✅ Step-by-step examples with numbered instructions
- ✅ Validation checkpoints ("Check your work" sections)
- ✅ Visual comparisons (markdown source vs rendered output)
- ✅ Clear success criteria for each exercise
- ✅ Frequent comprehension checks

**Validation Method**:
Manual review confirms scaffolding elements present in all lessons.

---

## 4-Stage Progression Validation

### Stage 1: Manual Foundation (Lesson 1)

**REQUIRED**:
- ✅ Students write markdown manually in text editor (no AI assistance)
- ✅ Manual validation exercises (students check their own syntax)
- ✅ Explicit instruction: "Open text editor, create file, write markdown manually"
- ✅ "Try With AI" focuses on VALIDATION (student created markdown, AI checks syntax) not GENERATION

**FORBIDDEN**:
- ❌ AI-assisted markdown generation in foundation lesson

---

### Stage 2: AI Collaboration (Lessons 2-3)

**REQUIRED Three Roles Demonstration**:
- ✅ **AI as Teacher**: AI suggests pattern student didn't know (explicit narrative)
- ✅ **AI as Student**: Student corrects AI with domain constraints (explicit narrative)
- ✅ **AI as Co-Worker**: Iteration produces better result than either alone (convergence loop)

**Detection Phrases**:
- "AI suggested..."
- "You refined by..."
- "Together you converged on..."

**Validation Method**:
Manual review confirms presence of all three roles in Lessons 2-3.

---

### Stage 3: Intelligence Design (Lesson 4)

**REQUIRED**:
- ✅ Students create reusable specification template
- ✅ Template uses Persona + Questions + Principles pattern
- ✅ Template applied to novel feature without lesson reference

**Template Structure**:
```markdown
## Persona
"Think like [role] ensuring [outcome]"

## Analysis Questions
1. [Question 1]
2. [Question 2]
...

## Principles
1. [Principle 1]
2. [Principle 2]
...

## Template
[Reusable structure]
```

---

### Stage 4: Spec-Driven Integration (Lesson 5)

**REQUIRED**:
- ✅ Specification written FIRST (before any implementation discussion)
- ✅ Capstone composes ALL skills from Lessons 1-4:
  - Headings (L1)
  - Lists (L2)
  - Code blocks (L3)
  - Links (L4)
- ✅ AI provides feedback on specification clarity
- ✅ Students refine specification based on feedback
- ✅ Exercise explicitly states: "Do NOT ask AI to implement code"

**Validation Method**:
Capstone rubric includes checklist for each markdown skill presence.

---

## Specification-Language Focus

### Code Block Context Validation

**REQUIRED**:
- ✅ All code blocks preceded by specification context:
  - "Expected system output:"
  - "API endpoint specification:"
  - "Feature requirements:"
  - "System behavior:"

**FORBIDDEN**:
- ❌ Code blocks showing implementation syntax
- ❌ Code blocks without specification context

**Validation Method**:
Manual review of all code blocks confirms specification context labels.

---

## Success Evals Mapping

### SC-001: Students write clear specifications
**Evidence**: Capstone deliverable includes complete task manager spec

### SC-002: 80% capstone success rate
**Evidence**: Capstone scope appropriate (3-4 features), validation checkpoints present

### SC-003: Distinguish specification from implementation
**Evidence**: Lesson 1 includes paired examples (spec vs code), students identify which is which

### SC-004: Three Roles demonstrated
**Evidence**: Lessons 2-3 narrative shows AI as Teacher/Student/Co-Worker

### SC-005: Zero code examples
**Evidence**: Grep validation returns zero matches for code patterns

---

## Anti-Pattern Detection

### Generic Markdown Tutorial Language (RED FLAGS)

**FORBIDDEN Phrases**:
- ❌ "Make your GitHub repository look professional"
- ❌ "Format your documentation"
- ❌ "Create pretty README files"

**REQUIRED Phrases**:
- ✅ "Communicate requirements to AI"
- ✅ "Structure your specifications"
- ✅ "AI agents parse this as..."

---

### Code-Centric Language (RED FLAGS)

**FORBIDDEN**:
- ❌ "Implement this feature"
- ❌ "Run this code"
- ❌ "Execute this command"

**REQUIRED**:
- ✅ "Describe WHAT system does"
- ✅ "Write specification for..."
- ✅ "Specify expected behavior"

---

## Validation Execution Checklist

### Pre-Implementation
- [ ] Spec.md reviewed and requirements clear
- [ ] Plan.md reviewed and lesson structure understood
- [ ] Constitutional frameworks internalized

### During Implementation (Per Lesson)
- [ ] Frontmatter includes: duration, concepts, proficiency, stage
- [ ] Concept count ≤ 7 (or 8 for capstone)
- [ ] No code examples present (grep check)
- [ ] Lesson ends with "Try With AI" ONLY
- [ ] No internal labels in student-facing text

### Post-Implementation (Cross-Lesson)
- [ ] All 5 lessons created
- [ ] Grep validation passes (zero code patterns)
- [ ] Lesson ending protocol validated (all end with "Try With AI")
- [ ] Internal label prohibition validated
- [ ] Three Roles demonstrated in L2-L3
- [ ] Template uses Persona+Q+P in L4
- [ ] Capstone composes all skills in L5
- [ ] CommonMark compliance validated
- [ ] SDD-RI methodology positioned (Intent Layer)

### Final Quality Check
- [ ] Run validation-auditor agent
- [ ] Create validation report
- [ ] Update chapter README with final titles
- [ ] Generate implementation summary

---

**This checklist ensures constitutional compliance and specification-language focus throughout Chapter 9 implementation.**
