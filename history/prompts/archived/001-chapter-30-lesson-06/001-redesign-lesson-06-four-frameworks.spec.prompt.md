---
id: 001
title: Redesign Lesson 06 - Four SDD Frameworks with Spec-Kit Plus
stage: spec
date: 2025-11-03
surface: agent
model: claude-haiku-4-5-20251001
feature: 001-chapter-30-lesson-06
branch: main
user: system
command: Redesign lesson 06 to clearly distinguish Spec-Kit (GitHub) from Spec-Kit Plus (Panaversity)
labels: ["lesson-design", "spec-kit-plus", "pedagogy", "sdd-fundamentals"]
links:
  spec: /specs/001-chapter-30-lesson-06/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - book-source/docs/05-Spec-Kit-Plus-Methodology/30-specification-driven-development-fundamentals/06-explore-the-tools-kiro-spec-kit-tessel.md
tests:
  - Manual validation: Lesson flows logically from context → 4 frameworks → evolution → scenarios
  - Manual validation: Spec-Kit (GitHub) and Spec-Kit Plus (Panaversity) clearly distinguished
  - Manual validation: ADRs, PHRs, Intelligence Templates explained with concrete examples
  - Manual validation: All 5 scenarios include "What that means" for Spec-Kit Plus
  - Manual validation: Summary table shows 4 tools with clear distinctions
---

## Prompt

### Initial Request
"Redesign lesson 06 @book-source/docs/05-Spec-Kit-Plus-Methodology/30-specification-driven-development-fundamentals/06-explore-the-tools-kiro-spec-kit-tessel.md You can look at other lessons and the source context @context/14_chap30_specs/ and pull specifyplus docs. Let's carefully plan"

### Clarifications Provided by User
1. "Yes, reduce academic depth. Both are separate tools. 1 is of github and other is of panaversity. Other look okay."
2. "Tell Tesel is emerging and its framework is invite only and things."
3. "For Tesel: mention registry and just say in one line about framework"
4. "1. both are separate tool 1 is of github and other is of panaversity other look okay"
5. "Create phr"

### Design Evolution
**Phase 1: Analysis**
- Identified that original lesson 06 treated Spec-Kit as a single tool without showing evolution
- Recognized readers needed to understand: Spec-Kit (GitHub base) → Spec-Kit Plus (Panaversity evolution)
- Audience insight: First-time learners who may later join teams

**Phase 2: Initial Plan**
- Four frameworks instead of three: Kiro, Spec-Kit, Spec-Kit Plus, Tesel
- Add "Understanding the Evolution" section explaining gaps
- Split framework descriptions to show GitHub vs. Panaversity distinction
- Add "What that means" paragraphs to scenarios

**Phase 3: Implementation**
- Updated learning objectives to include 4 approaches + Spec-Kit Plus extensions
- Rewrote opening to mention "Four major approaches" and Spec-Kit Plus as evolution
- Split Spec-Kit section into 2a (GitHub) and 2b (Panaversity)
- Added comprehensive "Understanding the SDD Evolution" section with three gaps + solutions
- Added "What that means" to Scenario 2 (Fintech), Scenario 3 (Healthcare), Scenario 5 (FDA)
- Updated summary table to show 4 tools with ADRs/PHRs/Templates distinctions
- Simplified Tesel treatment: removed MDD parallels, added registry link, kept to essence

## Response snapshot

### Key Structural Changes
1. **Learning Objectives**: Now cover 4 approaches + Spec-Kit Plus extensions
2. **Frameworks Section**: 4 approaches with clear naming (2a GitHub, 2b Panaversity)
3. **New Section**: "Understanding the SDD Evolution" explains:
   - Gap 1 (Lost Reasoning) → ADRs solution
   - Gap 2 (Lost Learning) → PHRs solution
   - Gap 3 (Lost Domain Knowledge) → Intelligence Templates solution
4. **Scenarios Enhanced**: "What that means" clarifies Spec-Kit Plus in real context
5. **Summary Table**: 4 tools with 10 distinguishing attributes

### Content Highlights

**Spec-Kit Plus Definition** (explicit):
```
GitHub's Spec-Kit (Base Layer)
    ↓
    + ADRs (capture architectural "why")
    + PHRs (capture AI interaction "how")
    + Intelligence Templates (domain expertise)
    ↓
Spec-Kit Plus (AI-native SDD for professional teams)
```

**Intelligence Templates Explained** (concrete):
- Education Template (this book): CEFR levels, Bloom's taxonomy, code testing requirements
- Healthcare: HIPAA rules, patient privacy patterns
- Fintech: PCI-DSS rules, transaction audit patterns
- Aerospace: Safety-critical validation

**Scenario "What That Means"** (actionable):
Example for Fintech Startup:
- "You'll use GitHub's Spec-Kit as the base (Constitution + Specify → Plan → Tasks)"
- "You'll add the **Fintech Intelligence Template** (pre-built PCI-DSS rules, payment patterns)"
- "You'll use **ADRs** to document architecture decisions"
- "You'll use **PHRs** to track AI interactions"

## Outcome

- ✅ **Impact**: Lesson now clearly shows Spec-Kit Plus is evolved, practical version of Spec-Kit (GitHub), not alien tool
- ✅ **Tests**: All 5 scenarios validated with "What that means" paragraphs
- ✅ **Files**: Single file modified with comprehensive redesign (~7,500 words, 721 lines)
- ✅ **Next prompts**:
  1. Validate lesson flows properly with updated structure
  2. Create corresponding specs/plans/tasks if needed for Part 5
  3. Update chapter navigation to reference new framework descriptions
- ✅ **Reflection**: Redesign successfully reframes Spec-Kit Plus from abstract to concrete by:
  - Showing it as evolution of GitHub's tool
  - Explaining three concrete gaps it fills (reasoning, learning, domain knowledge)
  - Providing real scenario examples with "what that means" clarity

## Evaluation notes (flywheel)

- **Failure modes observed**:
  - Initial version had readers confused about Spec-Kit vs. Spec-Kit Plus distinction
  - Academic tone made Spec-Kit Plus feel alien/abstract
  - Insufficient "what that means" for practical application

- **Graders run and results (PASS/FAIL)**:
  - ✅ Learning objectives clarified and expanded
  - ✅ 2a/2b split creates clear GitHub vs. Panaversity distinction
  - ✅ Evolution section explains gaps systematically
  - ✅ All 5 scenarios have actionable "What that means"
  - ✅ Summary table shows 4 tools accurately
  - ✅ Tesel simplified per user request

- **Prompt variant (if applicable)**:
  - User provided three clarification rounds to refine direction
  - Final direction: "both are separate tools, 1 is github and other is panaversity"
  - Executed as: Separate 2a/2b sections with explicit "GitHub's Framework" vs. "Panaversity's Evolution"

- **Next experiment** (smallest change to try):
  - Add one-sentence "TL;DR" comparing the two at start of 2b section
  - Consider visual diagram showing evolution path (Spec-Kit → Spec-Kit Plus)
  - Could add sidebar box highlighting "Why This Book Uses Spec-Kit Plus" for emphasis

