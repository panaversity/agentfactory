---
title: "Implement Phase - Execute Video Generation with AI"
chapter: 14
lesson: 8
duration_minutes: 120
proficiency_level: "B1"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Executing /sp.implement Command"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    measurable_at_this_level: "Student can orchestrate 12 atomic tasks for video generation using /sp.implement"

  - name: "Checkpoint Pattern Review"
    proficiency_level: "B1"
    category: "Soft"
    bloom_level: "Apply"
    measurable_at_this_level: "Student reviews output, approves, and directs next phase at each checkpoint"

  - name: "Playwright MCP Browser Automation Validation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Evaluate"
    measurable_at_this_level: "Student validates Playwright successfully navigates Gemini and persists sessions"

  - name: "Video Quality Validation Against Specification"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Evaluate"
    measurable_at_this_level: "Student verifies video matches specification (duration, codec, content)"

  - name: "Iteration and Debugging with AI"
    proficiency_level: "B1"
    category: "Soft"
    bloom_level: "Apply"
    measurable_at_this_level: "Student identifies failures, iterates with AI, and re-executes tasks"

learning_objectives:
  - objective: "Execute /sp.implement to orchestrate 12 atomic video generation tasks"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful implementation with visible checkpoints"

  - objective: "Validate AI-generated video against specification and quality criteria"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "ffprobe verification and playback validation"

  - objective: "Apply checkpoint pattern to maintain control during complex automation"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Evidence of review at each phase boundary"

  - objective: "Debug failures and iterate toward specification compliance"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successfully resolve issues and re-execute tasks"

cognitive_load:
  new_concepts: 8
  assessment: "8 new concepts (implement orchestration, checkpoint review, Playwright validation, video validation, iteration loops, session persistence, quality gates, artifact preservation) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Analyze implementation decisions captured in PHRs; modify video prompt for different product types"
  remedial_for_struggling: "Focus on Phase 1-2 (Setup/Session) before attempting video generation; validate with simplified success criteria"

generated_by: "content-implementer v1.0.0"
source_spec: "specs/013-chapter-14-redesign/spec.md"
source_plan: "specs/013-chapter-14-redesign/plan.md"
source_tasks: "specs/013-chapter-14-redesign/tasks.md"
created: "2025-11-25"
last_modified: "2025-11-25"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Implement Phase - Execute Video Generation with AI

Here's where theory becomes reality. You have a specification, a plan, and 12 atomic tasks. Now you'll execute them with your AI companion.

This lesson focuses on **control and validation**. The checkpoint pattern keeps you in charge while AI handles the heavy lifting. You'll review, approve, and direct—maintaining human judgment at every critical decision.

---

## What Happens During Implementation

When you run `/sp.implement`, the workflow follows this pattern:

```
Phase 1: Setup Verification
├─ Task 1.1: Verify Playwright MCP
├─ Task 1.2: Create session storage
├─ Task 1.3: Verify Gemini accessibility
└─ CHECKPOINT 1: Review setup outputs

Phase 2: Browser Session Establishment
├─ Task 2.1: First-run login (you enter credentials)
├─ Task 2.2: Verify session persistence
└─ CHECKPOINT 2: Review session file

Phase 3: Video Generation
├─ Task 3.1: Finalize video prompt
├─ Task 3.2: Login with saved session
├─ Task 3.3: Submit prompt to Gemini
├─ Task 3.4: Download generated video
└─ CHECKPOINT 3: Review video file

Phase 4: Validation & Completion
├─ Task 4.1: Move video to project directory
├─ Task 4.2: Verify format (ffprobe)
├─ Task 4.3: Test playback and content
└─ CHECKPOINT 4: Project complete
```

At each checkpoint, **you** decide whether to continue. This prevents wasting time on downstream tasks if an earlier phase has issues.

---

## Running /sp.implement

Start the implementation in Claude Code:

```
/sp.implement

Context:
- Specification: specs/013-chapter-14-redesign/spec.md
- Plan: specs/013-chapter-14-redesign/plan.md
- Tasks: specs/013-chapter-14-redesign/tasks.md

Execute Phase 1 (Setup Verification) with checkpoint.
```

The agent will:
1. Read your tasks.md
2. Execute Phase 1 tasks sequentially
3. Show outputs (verified configs, created directories)
4. Wait for your checkpoint review

---

## Checkpoint Review Pattern

**CHECKPOINT 1: After Phase 1 (Setup)**

Your AI companion reports:
```
✓ Playwright MCP verified in claude-config.json
✓ Session storage directory created at .session-storage/
✓ Gemini.google.com accessible (no geoblocking)

Ready for Phase 2: Browser session establishment
```

**Your review checklist:**
- [ ] Playwright MCP actually configured? (check file)
- [ ] Directory exists? (file explorer confirms)
- [ ] Can you see Gemini homepage in regular browser?
- [ ] Any errors or warnings?

**Your decision:**
- ✅ "Looks good. Committing Phase 1. Proceed to Phase 2."
- ❌ "Playwright not appearing in config. Let me fix and retry."

**What happens next:**
- You direct the next phase
- Agent doesn't continue autonomously
- You maintain control

---

## The Iteration Loop: What When Things Fail

**Real scenario**: You reach Phase 3 (video generation). Gemini's response timeout happens.

**Without checkpoints** (risky):
```
Agent: "I've completed everything. Phase 3 failed silently at task 3.3.
        Now I need to fix task 3.3, re-run 3.4, and validate everything
        again. I'm going to try this 3 times automatically."
You: "Wait, what exactly failed? Why are you retrying without asking?"
```

**With checkpoints** (controlled):
```
After Phase 2: You review, commit, approve Phase 3
After Task 3.2: You see "Login successful"
After Task 3.3: Timeout error appears
You: "Gemini timed out. Let me check the website manually...
      Gemini is actually running fine. Network issue on our end.
      Let's retry Task 3.3."
Agent: "Understood. Retrying Task 3.3 with extended timeout..."
Task 3.3 succeeds, continues to 3.4
```

You caught the network issue immediately, narrowed the failure to one task, and fixed it. Without checkpoints, the agent might waste 10 minutes retrying blindly.

---

## Phase 3 Details: The Video Generation Happens Here

This is where you see Playwright MCP in action. The agent:

1. **Uses saved session** to log back into Gemini (no manual login needed)
2. **Inputs your video prompt** into the Gemini chat interface
3. **Waits for generation** (typically 60-90 seconds)
4. **Downloads the video** when complete

Example terminal output you'll see:
```
Task 3.2: Playwright opening Gemini with saved session...
  ✓ Session cookies loaded from .session-storage/gemini-session.json
  ✓ Gemini.google.com loaded
  ✓ Verified logged-in state (no login form visible)

Task 3.3: Submitting video generation prompt...
  ✓ Chat input field focused
  ✓ Prompt pasted (847 characters)
  ✓ Submit button clicked
  ✓ Gemini displays "Generating video..."

Task 3.4: Waiting for generation...
  [60s] Generating... (progress indicator visible)
  [90s] Download button appeared
  ✓ Video downloaded: ~/Downloads/Gemini-Generated-Video-20251125.mp4
```

**This is the payoff moment.** A video file exists on your machine—generated by AI through browser automation orchestrated by specification.

---

## Validation Phase: Proving Your Video Meets Requirements

After generation, Phase 4 validates the video against your specification.

**Task 4.2: Verify File Properties**

The agent runs ffprobe to confirm video specifications:

```bash
ffprobe -v error -select_streams v:0 \
  -show_entries stream=codec_name,width,height,duration \
  ./output/demo-video.mp4
```

Expected output:
```
codec_name=h264
width=1920
height=1080
duration=48.0
```

**What you're checking:**
- ✅ Codec: h264 (web-compatible, as specified)
- ✅ Resolution: 1920x1080 (matches spec)
- ✅ Duration: 48 seconds (within 45-60s requirement)
- ✅ File size: > 2MB (indicates full render, not truncated)

**Task 4.3: Playback Validation**

You play the video and verify:
- Dashboard scene appears at start
- Sign-up button interaction visible
- Form filling animation smooth
- Success confirmation with confetti
- Text overlay: "Sign up in under 60 seconds"
- Background music present (no voiceover)
- No distorted frames or artifacts

---

## What You Get After Completion

The visible artifact:

```
./output/demo-video.mp4
├─ File size: 3.2 MB
├─ Duration: 48 seconds
├─ Codec: H.264
├─ Resolution: 1920x1080
└─ Content: Product demo matching specification
```

The invisible artifact:

```
history/prompts/chapter-14/
├─ 001-specify-phase.md (auto-captured)
├─ 002-clarify-phase.md (auto-captured)
├─ 003-plan-phase.md (auto-captured)
├─ 004-tasks-phase.md (auto-captured)
└─ 005-implement-phase.md (auto-captured your implementation decisions)
```

PHRs (Prompt History Records) capture every major decision during implementation. Later, when you create the `generate-video` skill in Lesson 09, these PHRs explain **why** you chose this architecture.

---

## Common Implementation Mistakes

### Mistake 1: Skipping Checkpoint Review

**The error**: Agent says "Phase 1 complete" and you immediately say "Next phase"

**Why it fails**: Hidden issues don't surface until Phase 3, when you've wasted time

**The fix**: Spend 2 minutes reviewing at each checkpoint:
- Does the output match the plan?
- Are there error messages?
- Does anything look suspicious?

### Mistake 2: Accepting Video Without Validation

**The error**: "Video generated! Project done!"

**Why it fails**: Video might be 15 seconds (truncated), wrong codec, or completely missing required scenes

**The fix**: Run ffprobe AND play the video. Both checks matter.

### Mistake 3: Not Iterating When First Attempt Fails

**The error**: "Gemini timed out. Oh well, the project failed."

**Why it fails**: Timeouts are often transient. Retrying works.

**The fix**: When Phase 3 fails, work with AI to diagnose:
- Network timeout? Retry with longer wait.
- Prompt too complex? Simplify and retry.
- Gemini down? Check manually, then retry.

---

## Try With AI

Ready to execute your video generation? Practice this workflow:

**Explore Checkpoint Review:**
> "I'm about to run `/sp.implement` for video generation. At each checkpoint, what should I review? For Phase 1: How do I verify Playwright MCP is actually configured? For Phase 2: What does a valid session file look like (where do I check the file size and cookie tokens)? For Phase 3: How do I know if video generation actually started vs. failed silently? Create a specific review checklist for each phase."

**Practice Validation:**
> "After the agent downloads the video to `./output/demo-video.mp4`, walk me through validation: (1) What ffprobe command verifies codec, resolution, and duration? (2) What does correct output look like? (3) What would indicate a problem (wrong resolution, truncated duration)? (4) How do I manually verify content (scenes present, timing correct, quality acceptable)?"

**Prepare for Failure:**
> "What could go wrong during video generation? (1) Gemini times out—how do I retry? (2) Video is 15 seconds instead of 45—what caused this? (3) Downloaded file is corrupted—how do I detect and fix? (4) Session expires—how do I re-login? For each scenario, outline steps to recover and continue."

**Reflect on Your Outcome:**
> "After generating my video, I'll have a tangible artifact: `./output/demo-video.mp4`. What does this file prove? How is this different from 'I learned about Spec-Kit Plus'? How would you explain to someone why specification-driven video generation is more reliable than 'just telling AI to make a video'?"

---
