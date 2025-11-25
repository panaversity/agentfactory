---
title: "Specify Phase - Writing Your Video Generation Specification"
chapter: 14
lesson: 4
duration_minutes: 90
proficiency_level: "B1"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Business-First Video Specification"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write specification defining video intent, constraints, and success criteria without implementation details"

  - name: "Evals-First Collaboration Pattern"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student conducts informal conversation with AI to clarify video requirements before formal specification"

  - name: "Video-Domain Success Criteria"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student identifies testable success criteria for video generation (duration, format, playability, file size)"

  - name: "Specification Template Completion"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student writes complete specification with Intent, Constraints, Success Evals, Non-Goals sections"

learning_objectives:
  - objective: "Conduct evals-first conversation with AI to clarify video success criteria before formal specification"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Completion of pre-specification dialogue with AI companion"

  - objective: "Write complete product demo video specification with clear intent, constraints, and success criteria"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Specification completeness review (all required sections included, no implementation details)"

  - objective: "Distinguish video success evals (marketing impact) from technical constraints (format, duration, file size)"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Accurate categorization of video requirements into evals vs constraints"

  - objective: "Identify edge cases and constraints specific to video generation (Gemini availability, file downloads, playback validation)"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Edge case documentation includes video-domain specific concerns"

cognitive_load:
  new_concepts: 8
  assessment: "8 new concepts (Evals-first pattern, Video-domain context, Intent definition, Constraint documentation, Success criteria, Non-goals, Specification structure, Business vs technical requirements) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Write specifications for 3 different video types (product demo, tutorial, social media clip); compare how requirements differ by format, duration, audience"
  remedial_for_struggling: "Use provided video specification template; fill in essential sections (Intent, Constraints, Success Evals) before moving to Clarify phase"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/chapter-14-video-pivot/spec.md"
created: "2025-11-25"
last_modified: "2025-11-25"
git_author: "Claude Code"
workflow: "manual-implementation"
version: "1.0.0"
---

# Specify Phase - Writing Your Video Generation Specification

Welcome to the most critical phase in SDD-RI: **Specification**. In the last lesson, you defined your project's global rules. Now you're writing the blueprint for a specific feature: a **product demo video**.

Here's the paradigm shift: In AI-native video generation, **your ability to describe what success looks like is more valuable than your ability to write video generation code**. Vague prompts produce mediocre videos. Clear specifications produce exactly what you need—the first time.

In this lesson, you'll learn how professionals write video specifications by practicing an evals-first approach: having a real conversation with AI to clarify what success means before writing the formal specification document.

---

## Understanding Video Success: Beyond "Good Looking"

Before writing a specification, professionals define what success actually means. This isn't technical validation ("Does the file download?"). This is **business validation**: "Does this video convert prospects into customers?"

### Why Evals-First Matters for Video

Consider these two approaches to the same problem:

**Without evals-first** (vague):
> "Create a product demo video showing our sign-up flow"

AI generates a video. But did you want:
- 30 seconds or 2 minutes?
- Silent or with voiceover?
- Professional animation or screen recording?
- Vertical (mobile) or horizontal (web)?

**With evals-first** (clear):
> "Create a 45-60 second product demo video. Success means:
> - Potential customers understand sign-up in under 60 seconds (target: 100+ views on YouTube)
> - Sign-up process is clearly visible (no blurry text, no fast-forwarding)
> - Video ends with call-to-action (CTA visible for 3+ seconds)
> - Marketing message: 'Sign up in under 60 seconds'"

Now AI knows exactly what you're optimizing for.

### The Evals-First Conversation Pattern

Here's how professionals work in practice:

**Step 1: You Drive the Exploration**

YOU ask questions to explore what success looks like. The AI helps you think through requirements—it doesn't interview you.

```
You: "I want to create a product demo video for our SaaS platform.
What makes a demo video successful for converting prospects?"

AI: "Great question. Successful demo videos typically:
- Show the core workflow (not all features)
- Demonstrate value in under 90 seconds (attention span constraint)
- Include clear call-to-action at the end
- Have good visual clarity (text readable, no fast-forwarding)

For your SaaS, what's the single most important workflow to demo?"

You: "Our sign-up flow. We want prospects to see how fast it is."

AI: "Perfect. So success metric: 'Prospects understand sign-up speed.'
How fast is it realistically? 30 seconds? 60 seconds?"

You: "Sign-up takes about 45-60 seconds end-to-end."

AI: "So your video should show:
- Start: User on landing page
- Middle: User filling out form (showing each field)
- End: User logged in, on dashboard
- Total duration: 45-90 seconds (matches actual speed + some context)"

You: "That works. What about quality? MP4 format? Resolution?"

AI: "For YouTube: 1080p MP4 is standard.
Duration: 45-60 seconds is optimal for YouTube recommendations.
File size: Keep under 50MB for fast upload.
For Gemini generation: We'll specify these as constraints."
```

**Step 2: Formalize Into Specification**

That conversation becomes the spec. The spec says: "Here's what success looks like for THIS video" (one feature). Now you can write it down with structure:

- **Intent**: What problem does this video solve? (Product demo for sign-up)
- **Constraints**: Technical requirements (MP4, 1080p, 45-60 seconds)
- **Success Evals**: How do we know it worked? (Marketing impact, clarity, CTA visible)
- **Non-Goals**: What are we NOT doing? (No post-editing, no custom animation)

---

## Writing Your Video Specification

Now it's your turn. Use the evals-first approach to write a specification for your product demo video.

### Step 1: Explore Success Criteria

Open your AI companion and have this conversation:

```
I'm writing a specification for a product demo video showing our [feature/workflow].

Before I write the formal spec, help me clarify what success means:

1. What's the single most important workflow to demonstrate?
2. How long should the video be? (30 sec? 60 sec? 2 min?)
3. Who's watching this video? (prospects? existing users? marketing team?)
4. What action do you want viewers to take after watching? (sign up? contact sales?)
5. What makes a demo video "clear"? (readable text, pacing, visual quality?)
6. What format and resolution do we need? (YouTube? LinkedIn? Email?)

[Have the conversation with your AI companion. Take notes on insights that emerge.]
```

### Step 2: Create Your Video Specification File

Based on your conversation, create a file called `specs/video-generation/product-demo.md` with this structure:

```markdown
# Specification: Product Demo Video

## Intent
Generate a [X] second product demo video showcasing [workflow name].
Target audience: [prospects/users/partners]
Marketing message: [one-sentence value proposition]
Platform: [YouTube/LinkedIn/website]

## Constraints
- Duration: [X-X seconds]
- Format: MP4, H.264 codec
- Resolution: 1080p (1920x1080), 30 FPS
- File size: < [X] MB (for fast upload)
- Tool: Gemini.google.com (free tier, browser-based)
- Playback validation: Must download and play without errors

## Success Evals (How We Know It Worked)
- Video file downloads successfully within 60 seconds
- Video duration matches specification (X-X seconds)
- Video plays without errors on standard players (VLC, YouTube, web browser)
- Core workflow is clearly visible (text readable, no fast-forwarding)
- Call-to-action is present and visible for 3+ seconds
- File is valid MP4 format

## Non-Goals (What We're NOT Building)
- Post-generation editing (no custom transitions, effects)
- Voiceover or background music (screen recording + silence)
- Multi-language versions (English only for MVP)
- Mobile-optimized vertical format (horizontal only)
- Custom branding animations (demo of core workflow only)
```

### Step 3: Review Your Specification

Ask yourself (or your AI companion):

- [ ] **Intent is clear**: Could someone outside your project explain what this video demonstrates?
- [ ] **Constraints are testable**: Can you verify duration, format, and playability?
- [ ] **Success Evals are measurable**: Can you check these without subjective judgment?
- [ ] **Non-goals are explicit**: Did you clearly state what's NOT in scope?
- [ ] **No implementation details**: Does the spec avoid "how to build" (that's the Plan phase)?
- [ ] **Video-domain specific**: Does this account for Gemini constraints, file downloads, playback validation?

### Step 4: Test Your Specification Quality

Ask your AI companion:

```
I've written this video specification:

[Paste your spec]

Can you review it for clarity?

1. What's ambiguous? (Where could I misunderstand what success looks like?)
2. What constraints are missing? (Duration? File size? Format validation?)
3. What edge cases might I encounter? (Gemini timeout? File download failure? Playback error?)
4. How would you describe this video to someone who hasn't read the spec?
```

---

## Specification Quality Indicators

### A Strong Specification

A strong video spec has:
- ✅ Clear intent (what workflow is being demonstrated)
- ✅ Explicit constraints (duration, format, file size, platform)
- ✅ Measurable success evals (testable, not subjective)
- ✅ Acknowledged non-goals (what's explicitly OUT of scope)
- ✅ No implementation details (no "use Gemini to..." or "prompt the AI to...")
- ✅ Video-domain awareness (file download, playback validation, Gemini free tier limits)

### A Weak Specification

A weak video spec has:
- ❌ Vague intent ("Create a demo video") - what workflow?
- ❌ Missing constraints ("Make it nice quality") - what's "nice"?
- ❌ Subjective evals ("Video should look professional") - professional to whom?
- ❌ No non-goals (scope creeps: "also add music", "also add text overlays")
- ❌ Implementation buried in spec ("Use Gemini's text-to-video feature with this prompt...")
- ❌ Ignores video realities (no mention of file downloads, Gemini timeouts, playback validation)

---

## Common Specification Mistakes

### Mistake 1: Vague Success Criteria

**The Error**:
```
Success Evals:
- Video looks professional
- Sign-up is clearly shown
- Viewers understand the product
```

**Why It's Wrong**: "Looks professional," "clearly shown," and "understand" are subjective. How would you verify them objectively?

**The Fix** (Make it measurable):
```
Success Evals:
- Video file downloads successfully within 60 seconds
- Text on screen is readable at YouTube standard quality (1080p)
- Sign-up form occupies 50%+ of video frame during input steps
- Video includes explicit call-to-action (visible for 3+ seconds)
- File plays without errors in 3+ video players (YouTube, VLC, browser)
```

### Mistake 2: Missing Video-Specific Constraints

**The Error**:
```
Constraints:
- Duration: 45-60 seconds
- Format: MP4
```

**Why It's Wrong**: You forgot critical video realities:
- How large should the file be? (Affects upload time)
- What if Gemini times out? (Affects retry strategy)
- How will you validate the video works? (Affects testing approach)

**The Fix**:
```
Constraints:
- Duration: 45-60 seconds
- Format: MP4, H.264 codec
- Resolution: 1080p (1920x1080), 30 FPS
- File size: < 50 MB (YouTube guideline)
- Frame rate: 30 FPS (standard for screen recording)
- Playback validation: Must download and play in at least 2 different players
- Gemini timeout: If generation exceeds 5 minutes, retry up to 3 times before failing
```

### Mistake 3: Implementation Leaking Into Specification

**The Error**:
```
Specification: Product Demo Video

...

Constraints:
- Use Gemini.google.com in an automated browser session
- Prompt Gemini with: "Generate a 60-second demo showing sign-up workflow"
- Capture browser output and save as MP4
```

**Why It's Wrong**: Specification defines WHAT. Implementation (Plan phase) defines HOW. Mixing them creates ambiguity.

**The Fix**:
```
Constraints:
- Must use browser-automation tool (Playwright or similar)
- Must use Gemini.google.com (free tier available)
- Generation timeout: 5 minutes maximum
- File must be downloadable directly (not copied manually)

[Implementation details go in the Plan phase, not here]
```

---

## Evals-First Conversation Example

Here's what a real evals-first conversation looks like:

```
You: "I want to create a demo video for our scheduling platform.
It's for prospects who haven't used the app yet. They need to see
how simple it is to schedule a meeting."

AI: "Great. A few clarifying questions:

1. What's the simplest happy path? (E.g., opening the app → creating one meeting?)
2. Should the video show the actual app or explain features verbally?
3. How much context do viewers need? (Sign-up → first meeting? Or assume they're already signed in?)
4. What's your call-to-action? (Sign up? Request a demo? Download?)
5. Where will this video live? (Website? YouTube? Email?)"

You: "1) Create one meeting from scratch (opening app to confirmation).
2) Show the app directly (no voiceover yet).
3) Assume they're signed in (skip login).
4) Call-to-action: 'Sign up free.'
5) On our website homepage."

AI: "Perfect. Based on that:
- Duration: 30-45 seconds is ideal for website embedding
- Format: MP4, but also consider WebM for better compression
- Resolution: 720p is fine for website (not YouTube)
- Call-to-action: Show as text overlay (3-5 seconds at end)

A few more constraints to nail down:

- Should the video loop? (Or does it play once?)
- What if they have multiple calendars? (Show selection or skip?)
- File size limit? (Affects compression)"

You: "Loop is fine. Skip calendar selection (assume default).
Keep under 10MB for fast loading on mobile."

AI: "Now I can help you structure the spec:

Intent: 45-second product demo showing scheduling workflow
Constraints:
  - 30-45 seconds duration
  - MP4 format, < 10MB
  - 720p resolution
  - Loops continuously on website

Success Evals:
  - Video downloads and plays in < 2 seconds on 3G connection
  - User sees full meeting creation workflow (open → select time → confirm)
  - Call-to-action visible for 4+ seconds
  - Video loops smoothly (no black frames between loops)

Non-Goals:
  - No voiceover (MVP is visual only)
  - No calendar selection (assumes default calendar)
  - No multiple user accounts (single user demo)"

You: "That's exactly what I need. Let me write that up formally."
```

Notice what happened: Through conversation, you moved from vague ("create a demo video") to crystal-clear requirements. AI didn't interview you—you drove the exploration, and AI helped you think through video-domain realities (duration for website, file size for mobile, loop behavior).

---

## Your Turn: Write and Refine

### Practice Exercise

1. **Have the conversation** (15 minutes)
   - Open your AI companion
   - Describe the video you want to create
   - Answer the clarifying questions above
   - Take notes on the requirements that emerge

2. **Write the specification** (20 minutes)
   - Create `specs/video-generation/product-demo.md`
   - Use the template structure (Intent, Constraints, Success Evals, Non-Goals)
   - Ensure constraints are testable, evals are measurable
   - Avoid implementation details

3. **Get feedback** (10 minutes)
   - Ask your AI: "Is this spec clear enough for someone to build a video from?"
   - Refine based on ambiguities AI identifies
   - Add any missing video-domain constraints

4. **Validate completeness** (5 minutes)
   - Check all required sections
   - Verify evals are measurable (not subjective)
   - Confirm no implementation details leaked in

---

## Try With AI

Ready to write your first complete video specification with AI collaboration? Practice the evals-first approach:

**Explore Success Criteria:**
> "I want to create a product demo video for my SaaS platform. Before writing the formal specification, help me clarify what success looks like. Ask me about: (1) What workflow am I demonstrating? (2) Who's the target audience? (3) Where will this video be used? (4) What should viewers do after watching? (5) What duration and format makes sense for that platform? Guide me through these questions so I understand what success means before I write the spec."

**Practice Video-Domain Constraints:**
> "Here are my video requirements: 45-60 second demo, MP4 format, 1080p resolution, YouTube upload. What constraints am I missing? Should I specify: file size limits? Frame rate? Playback validation approach? How Gemini generation failures should be handled? Help me identify constraints specific to video generation that I didn't think of."

**Test Specification Clarity:**
> "I've written a video specification at [path]. Can you review it and tell me: (1) What's ambiguous? (2) What video-domain constraints might I be missing? (3) What edge cases could happen during generation (Gemini timeout, file corruption, format issues)? (4) How would you build this video from my spec—would you have questions?"

**Apply to Your Feature:**
> "I need to write a specification for a [describe your video feature]. Walk me through the evals-first approach: (1) What problem does this video solve? (2) How do we measure success (marketing impact, clarity, viewer action)? (3) What are technical constraints (format, duration, file size, platform)? (4) What are we explicitly NOT building? Help me work through this conversation before I write the formal specification."

---