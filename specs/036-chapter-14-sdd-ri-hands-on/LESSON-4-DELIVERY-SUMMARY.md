# Lesson 4: Specify Phase - RECREATE Delivery Summary

**Lesson**: Chapter 14, Lesson 4 — "Specify Phase - Writing Your Video Generation Specification"

**File Location**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/04-specify-phase.md`

**Status**: COMPLETE (RECREATE strategy)

**Date Completed**: 2025-11-25

---

## What Was Delivered

### Complete Lesson Content (467 lines, ~3,500 words, 90-minute duration)

A fully reconstructed Lesson 4 focusing on **video specification writing** with integrated AI collaboration teaching. The lesson transforms students' understanding of specifications from generic principles (Chapter 13) into hands-on practice for a real project domain: AI-generated product demo videos.

---

## Key Content Sections

### 1. Conceptual Foundation: "Understanding Video Success"

- **Evals-First Paradigm**: Distinction between vague requests and clear specifications
- **Two Approaches Pattern**: Contrasting without/with evals-first conversation
- **Why Evals-First Matters**: Addresses video-specific ambiguities (duration, voiceover, format, orientation)
- **Conversation Pattern**: Shows how professionals work (exploration → formalization)

### 2. Hands-On Implementation: "Writing Your Video Specification"

**Step 1**: Explore Success Criteria

- Provides conversation starter with 6 clarifying questions
- Student drives exploration, AI provides video-domain insights

**Step 2**: Create Specification File

- Template with 4 required sections (Intent, Constraints, Success Evals, Non-Goals)
- Concrete variable placeholders for student customization
- Example structure using product demo context

**Step 3**: Review Your Specification

- 6-point checklist ensuring clarity and completeness
- Validation questions addressing video-domain concerns (Gemini constraints, file downloads, playback validation)

**Step 4**: Test Quality

- AI review prompts identifying ambiguities, missing constraints, edge cases

### 3. Quality Indicators & Common Mistakes

- **Strong vs Weak Specs**: Explicit comparison (clear intent vs vague intent, measurable evals vs subjective evals)
- **Mistake 1**: Vague success criteria (subjective → SMART transformation)
- **Mistake 2**: Missing video-specific constraints (demonstrates why Gemini timeout, file size, playback validation matter)
- **Mistake 3**: Implementation bleeding into specification (WHAT vs HOW separation principle)

### 4. Real-World Evals-First Example

- **Conversation**: 3-iteration dialogue showing natural progression from vague to clear
- **Video Domain Realism**: Addresses real constraints (website duration, file size for mobile, loop behavior)
- **Student-Driven**: Student asks clarifying questions, AI provides domain expertise
- **Natural Outcome**: Emerges from conversation without pedagogical labels

### 5. Student Practice Exercise

- **4-Step Structured Activity** (50 minutes total):
  1. Have actual conversation with AI companion (15 min)
  2. Write specification file (20 min)
  3. Get AI feedback (10 min)
  4. Validate completeness (5 min)

### 6. Try With AI (Closing Activity Only)

- **4 Copyable Prompts**:
  1. Explore Success Criteria (evals-first conversation)
  2. Practice Video-Domain Constraints (identify missing requirements)
  3. Test Specification Clarity (AI review pattern)
  4. Apply to Your Feature (custom video specification)
- **Expected Outcomes**: Specified for each prompt
- **No additional sections**: Lesson ends cleanly with Try With AI

---

## Constitutional Compliance

### Framework Invisibility (CRITICAL)

✅ **Zero Framework Labels**: All pedagogical scaffolding hidden from student view

- No "AI as Teacher/Student/Co-Worker" terminology
- No "What you learned" / "What AI learned" meta-commentary
- No "Three Roles Framework" labels
- No "Role 1:", "Role 2:", "Role 3:" divisions

✅ **Natural Narrative Only**: All AI collaboration shown through:

- Dialogue examples (conversation between student and AI)
- Action-based headings ("Step 1: You Drive the Exploration", "Evals-First Conversation Example")
- Outcome descriptions ("What happened:" section showing natural conversation progression)
- Practice prompts (copyable, action-focused)

### Layer 2 (AI Collaboration) Demonstration

✅ **AI as Teacher**: Suggests video-domain success criteria and constraints student might miss
✅ **AI as Student**: AI adapts recommendations based on student feedback (scheduling example)
✅ **AI as Co-Worker**: Convergence through iterative refinement (vague → clear transformation)
✅ **Framework Invisible**: Students EXPERIENCE Three Roles through collaborative dialogue, not studied pedagogy

### Structure Compliance

✅ **Ends with Try With AI ONLY**: Last major section is "## Try With AI" with no subsequent sections
✅ **No Bloat**: Zero "Key Takeaways", "Summary", "What's Next", "Congratulations" sections
✅ **Clean Closure**: Lesson ends with markdown delimiter after final AI prompt

---

## Proficiency Alignment (B1 - Intermediate)

| Dimension          | Status               | Details                                                              |
| ------------------ | -------------------- | -------------------------------------------------------------------- |
| **Cognitive Load** | ✅ 8/10 concepts     | Within B1 limit (7-10 concepts)                                      |
| **Scaffolding**    | ✅ Moderate          | Template provided, conversational approach, feedback mechanism       |
| **Bloom's Level**  | ✅ Apply + Analyze   | Write specification (Apply), distinguish evals/constraints (Analyze) |
| **Complexity**     | ✅ Real domain       | Video generation with Gemini constraints, not toy calculator         |
| **Prerequisites**  | ✅ Chapter 13 SDD-RI | Assumes students understand spec fundamentals, not first time        |

**8 Concepts Identified**:

1. Evals-first collaboration pattern
2. Video-domain context and constraints
3. Intent definition (WHAT not HOW)
4. Constraint documentation (technical realities)
5. Success evaluation criteria (measurable vs subjective)
6. Non-goals definition (scope boundary)
7. Specification structure (sections and relationships)
8. Business vs technical requirements separation

---

## Domain Expertise (Video Generation)

### Video-Specific Constraints Addressed

- Duration optimization (45-60 sec for YouTube, 30-45 sec for website)
- Format standards (MP4, H.264 codec, 1080p, 30 FPS)
- File size reality (upload speed, mobile loading, YouTube guidelines)
- Platform differences (YouTube vs website vs LinkedIn require different specs)
- Gemini free tier constraints (timeout policy, retry strategy, session persistence)
- File download validation (playback verification, format validation)

### Real-World Examples

1. **Sign-up Demo**: Product demo showing workflow speed as value proposition
2. **Scheduling Demo**: Platform feature demo for website embedding with specific constraints (loop behavior, file size for mobile)

Both examples show natural requirements evolution through conversation, not artificial constraints.

---

## Pedagogical Strengths

1. **Evals-First Pattern**: Core teaching is the CONVERSATION methodology, not just template completion
2. **Natural Collaboration**: Three Roles demonstrated through dialogue, not lectured
3. **Domain Specificity**: All examples use video generation context (not generic specs)
4. **Progressive Disclosure**: Vague → Clear transformation visible in multiple places (student's own video, example dialogues)
5. **Hands-On Structure**: Students conduct real activity (evals-first conversation with AI), not theoretical exercise
6. **Common Mistakes Framework**: Teaches through contrasting weak/strong approaches with transformation patterns
7. **Validation Loop**: Students get AI feedback and refine spec iteratively (mirrors professional workflow)

---

## Integration with Chapter 14

### Prerequisite (Lesson 3 - Constitution)

Sets global rules: video output format, Gemini constraints, testing requirements

### This Lesson (Lesson 4 - Specification)

Applies global rules to specific feature: product demo video requirements

### Next Phase (Lesson 5 - Clarify)

Refines specification using `/sp.clarify` command to identify gaps and ambiguities

**Cascade Pattern Visible**: Global rules (Constitution) → Feature spec (this lesson) → Refinement (Clarify) → Planning → Implementation

---

## Comparison to Previous Version

| Aspect               | Previous (Calculator Focus)      | Current (Video Focus)                                   |
| -------------------- | -------------------------------- | ------------------------------------------------------- |
| **Domain**           | Generic specification principles | Video-domain specific application                       |
| **Examples**         | Calculator operations            | Product demo videos (SaaS context)                      |
| **Context**          | Stand-alone concept lesson       | Chapter 14 hands-on project                             |
| **Constraints**      | Basic structure                  | Video-specific (Gemini, downloads, playback validation) |
| **Evals-First**      | Mentioned                        | Central teaching with dialogue examples                 |
| **AI Collaboration** | No explicit examples             | Real conversation examples showing Three Roles          |
| **Audience**         | Generic learners                 | Video project practitioners                             |

---

## Files Created/Modified

### Modified

- **04-specify-phase.md** — Complete RECREATE (467 lines, ~3,500 words)

### Verification Artifacts

- **LESSON-4-VERIFICATION-REPORT.md** — Constitutional compliance, content quality, pedagogical alignment
- **LESSON-4-DELIVERY-SUMMARY.md** — This document

---

## Quality Assurance

### Constitutional Compliance Verified

✅ Framework invisibility (zero role labels)
✅ Natural narrative only
✅ Ends with Try With AI ONLY
✅ No meta-commentary
✅ Layer 2 (AI Collaboration) demonstrated invisibly

### Cognitive Load Verified

✅ 8 concepts identified and counted
✅ Within B1 limit (7-10 concepts)
✅ Scaffolding appropriate for proficiency level
✅ Bloom's level (Apply/Analyze) matches tier

### Content Quality Verified

✅ Specification template complete (Intent, Constraints, Success Evals, Non-Goals)
✅ Common mistakes coverage (vague criteria, missing constraints, implementation bleeding)
✅ Real-world examples (sign-up demo, scheduling demo, student's own video)
✅ Practice exercise with clear steps and time estimates
✅ AI prompts are copyable and specific to video domain

### Integration Verified

✅ Prerequisite (Chapter 13 SDD-RI understanding) assumed
✅ Prerequisite (Lesson 3 Constitution) referenced
✅ Progression to Lesson 5 (Clarify) clear
✅ Chapter 14 context maintained throughout

---

## Next Steps (User Guidance)

### Immediate

1. Review verification report for any concerns
2. Commit lesson file to repository
3. Test Try With AI prompts with AI companion for clarity

### Optional Enhancement

1. Create `video-spec-template.md` in templates/ directory for easy student access
2. Add video-type comparison examples (different specs for product demo vs tutorial vs social media)
3. Expand Mistake 2 with more Gemini-specific constraints

### Downstream

- Lesson 5 (Clarify Phase) — Run `/sp.clarify` on student's video spec to identify gaps
- Lesson 6 (Plan Phase) — Develop Playwright implementation plan for video generation
- Lesson 7 (Tasks Phase) — Break implementation into atomic work units

---

## Summary

**Lesson 04: Specify Phase** has been successfully RECREATED for Chapter 14 (Spec-Kit Plus Hands-On) with:

- ✅ Complete video-domain specific content (467 lines, 90-minute duration)
- ✅ Constitutional compliance (framework invisible, ends with Try With AI only)
- ✅ B1 proficiency alignment (8 concepts, appropriate scaffolding, Apply/Analyze level)
- ✅ Layer 2 AI Collaboration demonstration (Three Roles visible through natural dialogue)
- ✅ Hands-on student practice (evals-first conversation, spec writing, validation loop)
- ✅ Real-world context (SaaS product demo videos with Gemini constraints)
- ✅ Integration with Chapter 14 progression (Constitution → Specification → Clarify → Plan → Tasks → Implement)

**Ready for immediate student use.**

---

**Created By**: content-implementer v1.0.0
**Creation Date**: 2025-11-25
**Workflow**: RECREATE (Lesson 04 specification writing, video-domain pivot)
