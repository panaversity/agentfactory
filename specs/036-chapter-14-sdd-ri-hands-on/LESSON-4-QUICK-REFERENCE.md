# Lesson 4: Specify Phase — Quick Reference

**File**: `apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/04-specify-phase.md`

**Status**: COMPLETE ✅

---

## What Students Learn

### Core Concept: Evals-First Specification Writing

Before formalizing requirements, professionals have a conversation with AI to clarify what success looks like. This lesson teaches that pattern through dialogue examples.

### Three Roles Framework (Invisible to Students)

Students experience AI collaboration through:

- **AI as Teacher**: Suggests video constraints they might miss (duration for platform, file size for performance)
- **AI as Student**: Adapts recommendations to their feedback (e.g., "For MVP simplicity, keep under 10MB")
- **AI as Co-Worker**: Converges on solution through iteration (vague → clear progression)

---

## Lesson Structure at a Glance

| Section                     | Purpose                   | Key Activity                      |
| --------------------------- | ------------------------- | --------------------------------- |
| Understanding Video Success | Why evals-first matters   | Compare vague vs clear requests   |
| Evals-First Conversation    | Show professional pattern | Dialogue example: vague → clear   |
| Writing Your Specification  | Hands-on application      | Step 1-4 guided practice          |
| Specification Quality       | Validation framework      | Checklist + AI review prompts     |
| Common Mistakes             | Learning through contrast | 3 mistake patterns with fixes     |
| Evals-First Example         | Real-world context        | Scheduling platform full dialogue |
| Your Turn: Write and Refine | Student practice          | 4-step exercise (50 min)          |
| Try With AI                 | Closing activities        | 4 copyable, specific prompts      |

---

## Key Video-Domain Context

### What Makes Video Specs Different

- **Platform matters**: 45-60s for YouTube, 30-45s for website, variable for email
- **File size matters**: 50MB for YouTube, <10MB for mobile website
- **Delivery matters**: Gemini free tier has timeouts, requires retry strategy
- **Validation matters**: Must verify downloadable, playable, format valid

### Constraints Students Learn to Specify

- Duration (seconds)
- Format (MP4, H.264, 1080p, 30 FPS)
- File size limit (affects compression, platform suitability)
- Playback validation (works in multiple players)
- Gemini timeout handling (free tier constraint)

### Success Evals (Measurable, Not Subjective)

- ✅ "Downloads in < 60 seconds" (measurable)
- ✅ "Plays without errors in YouTube, VLC, browser" (testable)
- ✅ "Text readable at 1080p" (verifiable)
- ✅ "CTA visible for 3+ seconds" (observable)
- ❌ NOT: "Looks professional" (subjective)
- ❌ NOT: "Viewers understand the product" (requires user testing)

---

## Constitutional Features

### Framework Invisibility

✅ Zero pedagogical labels exposed
✅ Natural dialogue examples only
✅ No "What you learned" meta-commentary
✅ No role labels (AI as Teacher/Student/Co-Worker)

### Structure Compliance

✅ Ends with "Try With AI" section ONLY
✅ No "Key Takeaways" or "Summary" sections
✅ No bloat or navigation helper text

### Teaching Strategy

✅ Layer 2 (AI Collaboration) demonstrated invisibly
✅ Three Roles visible through conversational examples
✅ Evals-first pattern is the core methodology
✅ Video-domain expertise integrated throughout

---

## Common Student Challenges (Addressed in Lesson)

### Challenge 1: Vague Success Criteria

**Problem**: "Video should look professional" (subjective)

**Solution in Lesson**: Mistake 1 section shows SMART transformation

- Specific: "Text readable at 1080p"
- Measurable: "CTA visible for 3+ seconds"
- Achievable: "Downloads within 60 seconds"
- Relevant: "Plays in YouTube, VLC, browser"
- Testable: Checkable without subjective judgment

### Challenge 2: Missing Video Constraints

**Problem**: Forgot file size, timeout handling, playback validation

**Solution in Lesson**: Mistake 2 section lists video-specific constraints

- File size (upload speed, platform limits)
- Gemini timeout (free tier reality)
- Playback validation (delivery assurance)
- Format verification (technical correctness)

### Challenge 3: Implementation Details in Spec

**Problem**: "Use Gemini.google.com in automated browser session" (HOW, not WHAT)

**Solution in Lesson**: Mistake 3 section separates WHAT (spec) from HOW (plan)

- Spec says: "Must use browser automation tool, Gemini.google.com, 5-minute timeout"
- Plan phase says: "Here's HOW we'll implement that" (Playwright MCP, session persistence, retry logic)

---

## Integration Points

### Prerequisites

- **Chapter 13**: SDD-RI fundamentals (specifications as primary design artifact)
- **Lesson 3**: Constitution (global project rules for video output)

### Foundational for Next Lessons

- **Lesson 5** (Clarify): `/sp.clarify` command refines the spec student writes here
- **Lesson 6** (Plan): Plan uses this spec to design implementation approach
- **Lesson 7** (Tasks): Tasks break down plan into atomic work units
- **Lesson 8** (Implement): Implementation generates video according to spec

---

## Key Metrics

| Metric                  | Value                                                        |
| ----------------------- | ------------------------------------------------------------ |
| Duration                | 90 minutes                                                   |
| Total Lines             | 467                                                          |
| Approximate Words       | 3,500                                                        |
| Learning Objectives     | 4                                                            |
| Cognitive Load          | 8 concepts (within B1 limit of 10)                           |
| Code/Dialogue Examples  | 4                                                            |
| Common Mistakes Covered | 3                                                            |
| Try With AI Prompts     | 4 copyable, specific prompts                                 |
| Practice Exercise Time  | 50 minutes structured activity                               |
| Proficiency Level       | B1 (Intermediate)                                            |
| Bloom's Levels          | Apply (write spec) + Analyze (distinguish evals/constraints) |

---

## Teaching Notes for Instructors

### What NOT to Do

- ❌ Don't expose pedagogical framework (framework invisibility required)
- ❌ Don't use calculator examples (video-domain specific context required)
- ❌ Don't skip evals-first conversation pattern (core teaching strategy)
- ❌ Don't make this only about template completion (conversation methodology is the point)

### What TO Emphasize

- ✅ Evals-first conversation shows how professionals think BEFORE writing
- ✅ Three Roles happen naturally through dialogue (not studied separately)
- ✅ Video-domain constraints are real (Gemini free tier, file downloads, playback validation)
- ✅ Specification quality cascades: unclear specs → poor plans → poor implementation

### Extension for Advanced Students

- Have them write specs for 3 different video types (product demo, tutorial, social media clip)
- Compare how requirements differ by format, duration, audience, platform
- Analyze how platform choice drives constraints (YouTube spec vs website spec vs email spec)

### Support for Struggling Students

- Provide `video-spec-template.md` with partial fills
- Focus on 3 essential sections first (Intent, Constraints, Success Evals) before Non-Goals
- Have them conduct evals-first conversation with instructor or peer before writing formal spec

---

## Quick Checklist for Student Success

Students should be able to:

- ✅ Explain why evals-first conversation happens before formal specification
- ✅ Conduct an evals-first conversation with AI about their video
- ✅ Write specification with Intent, Constraints, Success Evals, Non-Goals
- ✅ Distinguish measurable evals from subjective opinions
- ✅ Identify video-specific constraints (Gemini, downloads, playback, format)
- ✅ Recognize implementation details that belong in Plan, not Spec
- ✅ Use Try With AI prompts to get AI feedback on spec quality

---

**File Created**: 2025-11-25
**Reference Version**: 1.0.0
