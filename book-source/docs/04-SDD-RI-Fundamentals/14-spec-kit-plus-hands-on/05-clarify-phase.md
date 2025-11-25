---
title: "Clarify Phase - Refining Video Specs with /sp.clarify"
chapter: 14
lesson: 5
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Using /sp.clarify Command"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can run /sp.clarify on video specification and interpret feedback about Gemini constraints"

  - name: "Identifying Video-Specific Specification Gaps"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student recognizes ambiguities in video requirements (resolution, format, quality), missing Gemini assumptions, incomplete edge cases"

  - name: "Iterative Specification Refinement for AI-Generated Video"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student revises video specification based on /sp.clarify feedback and makes critical decisions about Gemini constraints vs desired outcomes"

  - name: "Addressing Technology Constraints in Specifications"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student identifies where Gemini free tier limitations affect spec (session timeout, format availability) and documents fallback behaviors"

learning_objectives:
  - objective: "Use /sp.clarify command to analyze video specification for gaps and video-specific edge cases"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful execution of /sp.clarify on video spec and interpretation of video-domain feedback"

  - objective: "Identify and resolve specification ambiguities in video generation (quality, format, duration, interactions)"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Refined specification addressing video-specific gaps (resolution, frame rate, Gemini constraints)"

  - objective: "Understand how /sp.clarify identifies technology-specific edge cases that affect specifications"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of how clarification surfaced Gemini availability or session timeout concerns"

cognitive_load:
  new_concepts: 6
  assessment: "6 new concepts (Clarify workflow, video-specific gap detection, Gemini constraint handling, quality ambiguity resolution, edge case identification, technology fallback decisions) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Run /sp.clarify multiple times on video spec; document spec evolution; create contingency plans for Gemini unavailability; design fallback video strategies"
  remedial_for_struggling: "Focus on top 3-4 clarifying questions from /sp.clarify; resolve those before moving to planning (ignore advanced Gemini optimizations for now)"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "Chapter 14 Spec-Kit Plus"
created: "2025-11-25"
last_modified: "2025-11-25"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Clarify Phase - Refining Video Specs with /sp.clarify

You've written a video generation specification. It looks good. But there are always gaps you didn't catch—ambiguities that seemed clear in your head but are actually unclear on paper. Edge cases specific to Gemini video generation that you missed. Assumptions about video quality, format, or timing you didn't state explicitly.

This is where the `/sp.clarify` command helps. **Clarify is NOT a formal workflow phase** like `/sp.specify` or `/sp.plan`. It's a **quick check** that your specification is complete before moving to planning.

Think of `/sp.clarify` as your AI companion putting on a "devil's advocate" hat and asking: "Wait, what about Gemini's session timeout? What if the video is shorter than you expected? What happens if the free tier can't generate MP4?" It finds gaps you might have missed, then you decide whether to update your spec.

The goal: Make your specification **so clear** that the planning phase can generate a perfect plan for Playwright MCP integration.

---

## What Does /sp.clarify Do?

### The Clarify Command

`/sp.clarify` analyzes your specification and reports:

1. **Ambiguous Terms** - Words that could mean multiple things in video context

   - Example: "professional quality video" (1080p or 720p? 30fps or 60fps? Which codec?)
   - Example: "clear sign-up flow demonstration" (how many clicks? what transitions?)

2. **Missing Assumptions** - Things you assumed but didn't state

   - Example: You assumed Gemini would generate MP4, but didn't state fallback if only WebM available
   - Example: You assumed video duration would be exactly 45 seconds, but Gemini might generate 40s or 50s
   - Example: You assumed free tier Gemini can handle your prompt, but didn't specify constraints or rate limits

3. **Incomplete Requirements** - Scenarios or cases you didn't cover

   - Example: You specified video generation but didn't specify download timeout handling
   - Example: You specified single interaction flow but didn't specify multi-step demo structure
   - Example: You specified success case but didn't specify Gemini unavailability fallback

4. **Technology Constraint Conflicts** - Where spec assumes capabilities Gemini may not have

   - Example: Spec says "generate 2-minute video" but free tier may have duration limits
   - Example: Spec says "use advanced animations" but Gemini may have styling constraints
   - Example: Spec says "guaranteed fast generation" but API rate limits may cause delays

5. **Edge Case Gaps** - Cases that should be handled but aren't documented
   - Example: Session timeout after how long? (60s? 90s? 120s?)
   - Example: What happens if Gemini generates video shorter than specified?
   - Example: What if download fails mid-transfer? Retry strategy?
   - Example: What video formats does Gemini support, and what's your fallback?

#### 💬 AI Colearning Prompt
> "Why is /sp.clarify valuable for video generation specs specifically? What kinds of gaps do AI video tools create that you might not anticipate?"

#### 🎓 Expert Insight
> Video generation introduces technology-specific edge cases that traditional specs miss: format compatibility (MP4 vs WebM), API rate limits, session timeouts, free-tier constraints. Gemini has constraints that affect what you can ask it to generate. Clarify surfaces these upfront, preventing implementation surprises. Two rounds of clarification produce video specs that Playwright MCP can execute reliably rather than failing mid-generation because of an unstated assumption about Gemini behavior.

---

## Part B: The Clarify Workflow

Here's how clarification works in practice for your video specification.

### Step 1: Run /sp.clarify

In Claude Code, from your video-project directory:

```
/sp.clarify

My video generation specification is at specs/video-generation/spec.md
Please analyze it for:
1. Ambiguous terms in video requirements (what does "professional quality" mean? resolution? frame rate?)
2. Missing assumptions about Gemini video generation (duration limits? format constraints? session timeout behavior?)
3. Incomplete requirements (what happens if video is shorter than expected? what format fallbacks?)
4. Technology-specific edge cases (Gemini unavailability? rate limiting? download failure handling?)
5. Conflicts between spec and Gemini free-tier capabilities

What gaps should I address before planning the Playwright MCP implementation?
```

Your companion will analyze your specification, identify video-generation-specific gaps or ambiguities, and ask clarifying questions. Review its findings and update your specification accordingly.

### Step 2: Re-Run /sp.clarify (Optional)

If you made significant changes to address Gemini constraints or edge cases, run `/sp.clarify` again:

```
I've updated my video specification based on your feedback.
Please analyze it again for remaining gaps.
Should I proceed to planning with Playwright MCP, or are there more clarifications needed?
```

Most video specifications need 1-2 clarification rounds. After that, they're ready for planning.

---

## Clarify Your Video Specification

Now let's clarify YOUR video specification—the one you wrote in Lesson 04.

### Step 1: Run /sp.clarify on Your Video Specification

In Claude Code, from your video-project directory, run:

```
/sp.clarify

My video generation specification is at specs/video-generation/spec.md

Please analyze it for:

1. AMBIGUOUS TERMS
   - What does "professional quality" mean? (1080p? 720p? codec? bitrate?)
   - What does "smooth transitions" mean? (fade? dissolve? duration?)
   - What does "clear sign-up flow" mean exactly? (how many steps? which UI elements?)

2. MISSING GEMINI ASSUMPTIONS
   - What video formats can Gemini generate? (MP4? WebM? Others?)
   - What are the session timeout constraints? (timeout after how long of inactivity?)
   - What are the rate limits? (videos per minute? concurrent requests?)
   - What's the maximum video duration Gemini can generate?

3. INCOMPLETE EDGE CASES
   - If Gemini generates video shorter than expected, what do I do?
   - If desired video format isn't available, what's my fallback?
   - If download fails mid-transfer, what's my retry strategy?
   - If Gemini is unavailable, what's my fallback behavior?

4. TECHNOLOGY CONFLICTS
   - Does my spec assume capabilities Gemini free tier doesn't have?
   - Are there timing or performance assumptions that conflict with API constraints?
   - Does my spec require sustained API usage that might trigger rate limiting?

List any gaps or questions. Which ones are CRITICAL (block implementation) vs NICE-TO-HAVE (improve quality)?
```

### Step 2: Verify Readiness

Ask your AI companion:

```
Based on the clarification feedback, is my video specification now ready for the planning phase?
What's the minimum set of clarifications I need to address before proceeding?
Which suggestions can I defer to future iterations?
```

---

## Cascade Quality Validation

Now test the cascade effect: Does your clarified specification improve Playwright MCP planning potential?

**Ask yourself**:

- ✅ Can a developer read your spec and understand exactly what Gemini needs to generate?
- ✅ Are Gemini's constraints and limitations explicitly documented?
- ✅ Are all video quality parameters defined (resolution, frame rate, codec, duration)?
- ✅ Are fallback behaviors documented for Gemini unavailability or format incompatibility?
- ✅ Is the interaction flow specific enough for Playwright to automate it?
- ✅ Are edge case handling strategies explicit (timeout behavior, retry logic)?

If yes to most, your spec is ready for planning.

---

## Common Mistakes

### Mistake 1: Skipping /sp.clarify Because "Spec Looks Good to Me"

**The Error**: "I wrote a detailed video spec. I don't need clarification."

**Why It's Wrong**: Video generation introduces technology-specific ambiguities. What's "obvious" to you (1080p quality? MP4 format? 45 second duration?) may be unclear or impossible given Gemini's actual constraints.

**The Fix**: Always run `/sp.clarify`. You'll be surprised what video-specific gaps emerge. Most video specs need 1-2 clarification rounds.

### Mistake 2: Ignoring Gemini Constraints Because "I'll Figure It Out Later"

**The Error**: "My spec doesn't specify video format or duration limits. I'll handle that during implementation."

**Why It's Wrong**: Unaddressed constraints create implementation surprises (Gemini can't generate your requested format, session times out before video completes, free tier has undocumented rate limits). This breaks planning.

**The Fix**: Address critical constraints upfront:
- "Gemini generates MP4; if unavailable, accept WebM"
- "Maximum 90-second video duration (Gemini constraint)"
- "If session times out after 120 seconds, retry up to 3 times"

### Mistake 3: Accepting All AI Suggestions Without Critical Thinking

**The Error**: AI says "Consider adding format validation" → immediately adding it without evaluating necessity

**Why It's Wrong**: Not all suggestions improve your spec. Some add unnecessary complexity or over-engineer for a simple case.

**The Fix**: Evaluate each suggestion:
- Is this edge case likely in your actual use case?
- Does handling this improve clarity or add noise?
- Can I defer this to version 2?

Then decide: Accept, Reject, or Modify.

#### 🤝 Practice Exercise

> **Ask your AI**: "I ran /sp.clarify on my video specification and received 6-8 suggestions for improvement. Help me categorize them: (1) Which are CRITICAL (Playwright implementation won't work without them)? (2) Which are NICE-TO-HAVE (improve robustness but not blocking)? (3) Which can I defer to future iterations? Then explain how addressing the critical gaps will help the planning phase generate a better Playwright MCP implementation."

**Expected Outcome**: Your AI should categorize clarification feedback by urgency (e.g., "Gemini format compatibility = critical, retry logic optimization = nice-to-have"), explain why critical gaps block planning (unclear Gemini constraints → unclear Playwright automation approach), and help you make informed decisions about which changes to prioritize before moving to `/sp.plan`.

---

## Try With AI

Ready to validate your clarified video specification and ensure it's ready for Playwright MCP planning? Test your improvements:

**🔍 Explore Video-Specific Ambiguities:**
> "Compare my original video spec with the clarified version. What video-specific ambiguities were resolved? Show me exact examples: (1) Where I clarified video quality requirements, (2) Where I documented Gemini format constraints, (3) Where I added edge case handling. Are there still any vague video requirements remaining?"

**🎯 Practice Technology Constraint Handling:**
> "Imagine you're implementing this spec with Playwright MCP. Read my clarified video specification and tell me: (1) Can you automate the sign-up flow from this spec alone without asking questions? (2) What are the Gemini constraints you need to know about? (3) What edge cases need explicit handling? (4) Is the specification complete enough to write tests before implementation?"

**🧪 Test Prioritization for Video Edge Cases:**
> "I received 7 clarification suggestions about my video spec: [describe them]. Help me prioritize for Playwright MCP implementation: Which are CRITICAL (implementation won't work without them)? Which are NICE-TO-HAVE (improve quality but not blocking)? Which can I defer to later? Explain how each critical gap affects Playwright automation planning."

**🚀 Apply to Your Video Spec:**
> "I ran `/sp.clarify` on my video generation specification. The feedback focused on [describe areas: Gemini constraints? Format compatibility? Session timeout handling?]. Help me decide which clarifications to implement now vs later. What's the minimum set of changes needed to make my spec ready for Playwright MCP planning? Walk me through your decision framework for video-specific edge cases."

---
