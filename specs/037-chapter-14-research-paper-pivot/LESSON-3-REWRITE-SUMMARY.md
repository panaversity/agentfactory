# Lesson 3: Constitution Phase — Rewrite Summary

**Date**: 2025-11-26
**File**: `apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/03-constitution-phase.md`
**Version**: 2.0.0 (Complete rewrite)

---

## Changes Made

### Removed Content (All video-specific references)

- ❌ Video generation focus throughout lesson
- ❌ Gemini.google.com constraints (free tier limits, session handling)
- ❌ Playwright MCP references (browser automation, session persistence)
- ❌ Video quality standards (MP4 format, H.264 codec, resolution, frame rate)
- ❌ Video download validation and playback testing
- ❌ MCP references and tool-specific constraints
- ❌ Video project examples and use cases
- ❌ Gemini timeout retry logic (3 retries, rate limiting)

**Verification**: Grep for "video|Gemini|Playwright|MCP" returns 0 matches in rewritten file.

### Added Content (Research paper focus)

#### 1. Frontmatter Updates

```yaml
title: "Constitution Phase — Project-Wide Quality Standards"
chapter: 14
lesson: 3
duration_minutes: 30
proficiency_level: "A2"
cognitive_load:
  new_concepts: 4
```

Updated all metadata to remove video references; proficiency set to A2 (hands-on application learners).

#### 2. New Four Core Concepts

The lesson now teaches 4 new concepts:

1. **What is a Constitution** (project-wide rules, one per project)

   - Applied to research paper project
   - Distinguished from Specification (one feature)
   - Examples: citation accuracy, source verification, writing clarity

2. **The Cascade** (how Constitution guides downstream phases)

   - Shows concrete flow: Constitution → Specification → Plan → Tasks → Implementation
   - Strong Constitution produces strong downstream work
   - Weak Constitution produces vague downstream work

3. **One-Time vs Repetitive** (Constitution written once, then reused)

   - Pattern: Write Constitution → Commit to git → FOR EACH PAPER: Specify/Plan/Implement
   - Best practice workflow explained

4. **`/sp.constitution` Command Usage** (interactive guide tool)
   - What command does: asks targeted questions
   - What command does NOT do: write Constitution for you, assume project type, copy templates blindly
   - Constitution structure: Core Principles, Quality Standards, Technical Requirements, Constraints, Success Criteria

#### 3. New Part Structure

**Part A: Understanding `/sp.constitution` Command** (before creating)

- What the command does
- Constitution structure (5 sections)
- Expert insight on leverage and precision

**Part B: Create Your Research Paper Constitution** (hands-on creation)

- Step 1: Plan Constitution (5 minutes pre-work)
- Step 2: Run `/sp.constitution` with example research paper project prompt
- Step 3: Review and improve Constitution (check for testability, constraints, measurability)
- Step 4: Commit to git (immutability, clarity, traceability)

**Part C: How Constitution Guides Downstream Phases** (cascade demonstration)

- Constitution → Specification Phase (inherited standards, no re-specification)
- Constitution → Plan Phase (verification steps built in)
- Constitution → Task Phase (all tasks respect Constitution constraints)
- Constitution → Implementation Phase (AI interactions respect Constitution requirements)

#### 4. Research Paper Practice Vehicle

**Constitution Example** (ALL research papers):

```
✅ "All papers must cite primary sources, not just secondary sources"
✅ "All claims must be verified against authoritative sources"
✅ "Writing must be clear and direct (Flesch-Kincaid grade 10-12)"
✅ "All papers reviewed for plagiarism before submission"
✅ "All references formatted in APA style"
```

**Specification Example** (specific paper):

```
✅ "This paper explores AI development methodology"
✅ "Target length: 5,000 words"
✅ "Minimum 12 peer-reviewed sources required"
✅ "Due date: December 15"
✅ "Thesis: AI-native development requires thinking in specifications, not code"
```

**Why Research Paper?**

- Clear structure (Introduction, Literature Review, Analysis, Conclusion)
- Measurable success criteria (word count, sources, plagiarism check)
- Iteration loops (draft → fact-check → plagiarism scan → review)
- Universal (applies to ALL academic contexts, unlike video generation)
- Production-relevant (every developer writes documentation, specs, technical reports)

#### 5. Common Mistakes (Research Paper Specific)

1. **Too General Constitution** (vague like "good quality")

   - Fix: Testable criteria (plagiarism check, source verification, Flesch-Kincaid 10-12)

2. **Too Specific Constitution** (applies only to one paper)

   - Fix: Write Constitution for ANY paper in the project

3. **Forgetting to Commit Constitution**
   - Fix: Create → Commit → THEN /sp.specify

#### 6. Try With AI Section (Rewritten)

Four hands-on prompts using research paper context:

1. **Explore Cascade Effect**: Show how Constitution rules cascade through phases
2. **Test Constitution Completeness**: Review Constitution for testability and coverage
3. **Validate Against Specification**: Check if Constitution guides paper specs
4. **Apply to Your Domain**: Draft Constitution for different research types

---

## Pedagogical Framework

### Layer Identification

**Layer 2 (AI Collaboration)**

- Students have learned `/sp.constitution` in Chapter 13 (theory)
- Now applying it hands-on with research paper project
- Demonstrates bidirectional learning: student defines standards, AI creates Constitution, student refines

### Teaching Strategy

- **Opening Hook**: Constitution answers "What standards apply to ALL my work?"
- **Conceptual Foundation**: Distinguish Constitution (global rules) from Specification (feature-specific)
- **Cascade Visualization**: Show how Constitution flows into all downstream phases
- **Hands-On Practice**: Create real Constitution for research paper
- **Verification**: Commit to git as immutable baseline
- **Integration**: Show cascade through Specification/Plan/Tasks/Implementation

### Complexity Tier

**A2 Proficiency** (4 new concepts)

- Concept 1: What is Constitution (global rules)
- Concept 2: Constitution vs Specification distinction
- Concept 3: Cascade through downstream phases
- Concept 4: `/sp.constitution` command usage

Within A2 limit of 5-7 concepts per lesson.

---

## Content Quality Checks

### Testability

All standards examples use testable criteria:

- ✅ "Flesch-Kincaid grade 10-12" (measurable)
- ✅ "Zero plagiarism tolerance before submission" (verifiable)
- ✅ "APA citations" (checkable)
- ✅ "Minimum 15 sources, 50% peer-reviewed" (countable)

NOT vague like "good quality" or "well-researched"

### Completeness

Lesson covers:

- ✅ What Constitution is (definition, scope, purpose)
- ✅ How `/sp.constitution` command works (what it does, structure)
- ✅ How to create Constitution (plan, run command, review, commit)
- ✅ How Constitution guides downstream work (cascade with examples)
- ✅ Common mistakes (with fixes)
- ✅ Practice with "Try With AI"

### Alignment

- Aligned to learning objectives (use `/sp.constitution`, understand cascade, write testable criteria, distinguish Constitution vs Specification)
- Aligned to Chapter 14 context (hands-on Spec-Kit Plus workflow, research paper vehicle)
- Aligned to constitutional principles (specification primacy, progressive complexity, minimal content)

---

## File Details

**Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/03-constitution-phase.md`

**Size**: 487 lines (489 lines before metadata stripping, removed ~150 lines of video content, added ~140 lines of research paper content)

**Status**: Complete and ready for review

**Next Steps**:

1. User review of lesson structure and teaching approach
2. Validate against Chapter 14 learning outcomes
3. Commit to feature branch
4. Create PR for validation auditor review
