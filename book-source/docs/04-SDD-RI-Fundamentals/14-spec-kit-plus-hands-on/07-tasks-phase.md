---
title: "Tasks Phase - Atomic Work Units and Checkpoints"
chapter: 14
lesson: 7
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Using /sp.tasks Command"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can run /sp.tasks to decompose plan into atomic work units"

  - name: "Understanding Atomic Task Definition"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student understands atomic task = 15-30 minute unit with single acceptance criterion"

  - name: "Recognizing Task Dependencies"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student understands which tasks must complete before others (dependency graph)"

  - name: "Human-Controlled Checkpoint Pattern"
    proficiency_level: "B1"
    category: "Soft"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student understands Agent→Review→Commit→Continue workflow and human's role in each checkpoint"

  - name: "Playwright MCP Task Execution"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can recognize browser automation tasks using Playwright MCP"

  - name: "Video Prompt Engineering"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student understands structure of effective prompts for AI video generation (scene-by-scene, timing, style, constraints)"

learning_objectives:
  - objective: "Use /sp.tasks to decompose video generation plan into atomic work units"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful execution of /sp.tasks and understanding of task breakdown"

  - objective: "Understand checkpoint pattern for video generation: Setup → Session → Generation → Validation"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of checkpoint pattern and human's control role"

  - objective: "Create production-ready video generation prompt with scene-by-scene structure and timing"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Ability to write prompt with explicit scene structure, durations, and style keywords"

  - objective: "Trace requirement lineage from specification through plan to tasks"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Ability to follow a requirement from spec to task unit"

cognitive_load:
  new_concepts: 8
  assessment: "8 new concepts (Tasks command, atomic units, dependencies, checkpoint pattern, video prompts, Playwright MCP, session persistence, quality gates) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Design video prompts for 2-3 different product types (SaaS, mobile app, API); analyze how prompt structure changes by domain"
  remedial_for_struggling: "Focus on Phase 1 (Setup) and Phase 3 (Video Generation) tasks; delay Phase 4 validation until initial success"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/chapter-14-redesign/spec.md"
source_plan: "specs/chapter-14-redesign/plan.md"
created: "2025-11-25"
last_modified: "2025-11-25"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Tasks Phase - Atomic Work Units and Checkpoints

You now have:
- ✅ A clear video generation specification
- ✅ A detailed Playwright MCP implementation plan
- ✅ Documented architecture decisions (ADRs)

Next: Break the plan into **atomic work units** (tasks) that you'll execute. Each task is 15-30 minutes, has one acceptance criterion, and produces a verifiable output.

This lesson teaches the **checkpoint pattern**—the critical workflow practice that keeps YOU in control. The pattern is:

```
Agent: "Here's Phase 1 - Setup complete"
You: "Review... looks good!"
You: "Commit to git"
You: "Tell me what's next"
Agent: "Phase 2 - Session Setup starting"
```

NOT:

```
Agent: "Here's everything at once" (no human control)
```

The difference is huge. Checkpoints keep you in control and catch issues early before wasting time on downstream tasks.

---

## What Are Tasks?

A **task** is a unit of work that:
- Takes 15-30 minutes to complete
- Has a single, clear acceptance criterion
- Depends on specific other tasks
- Produces one verifiable output (file, directory, validated state)

### Task Properties

**Size**: 15-30 minutes
- Too small (under 10 minutes) = too many micro-tasks, checkpoint overhead
- Too large (over 45 minutes) = hard to review, hard to fix if wrong

**Criterion**: Single, testable
- "Verify Playwright MCP configured" ✅
- "Verify MCP AND create directory AND test access" ❌ (three things)
- "Do setup stuff" ❌ (untestable)

**Independence**: Can be reviewed individually
- Doesn't require other tasks to be done first
- Or clearly depends on specific other tasks

**Clarity**: Defines exact acceptance criterion that signals completion
- ✅ "Session file exists at `.session-storage/gemini-session.json` AND contains valid cookies"
- ❌ "Session is working"

---

## The Checkpoint Pattern (CRITICAL)

This is **the most important concept** in this lesson. The checkpoint pattern is how you maintain control of the workflow.

### Pattern Definition

```
Loop:
  1. Agent: "I've completed Phase X (description of output)"
  2. Human: "Review the work (output visible and testable)"
  3. Human: "APPROVE" → Commit to git
  4. Human: "Tell me next phase"
```

### Why Checkpoints Matter

**Without Checkpoints** (dangerous):
```
You: "Generate a video for my product"
Agent: "Done! I've completed 12 tasks, logged into Gemini,
        navigated through 10 pages, generated video,
        moved it to output. All automated. You're welcome."
You: "Wait, which step failed? How do I fix it?
      I can't see what happened!"
Agent: "Already committed. Sorry! Rollback the whole thing?"
```

**With Checkpoints** (controlled):
```
You: "Start video generation workflow"
Agent: "Phase 1 (Setup) complete:
        ✓ Playwright MCP verified
        ✓ Session directory created
        ✓ Gemini.google.com accessible
        Ready for review."
You: "Read setup results... all looks good. Commit. What's next?"
Agent: "Phase 2 (Session Setup) - First run login..."
You: "I'm logging in now. Continue when ready."
Agent: "Phase 3 (Video Generation) - Here's the prompt..."
You: "Found issue with prompt timing. Fixing..."
Agent: "Phase 4 (Validation) - Video complete. Running checks..."
You: "Video validated. All quality gates pass. Project complete!"
```

### Your Role in Each Checkpoint

**Step 1: Human Reviews**
- See the actual output (file, directory structure, logged state)
- Ask: "Does this match the plan?"
- Ask: "Are there issues I should fix before continuing?"
- Ask: "Is this ready for the next phase?"

**Step 2: Human Decides**
- Approve ("Looks good, commit")
- Reject ("Fix this issue before continuing")
- Request clarification ("Explain this step")

**Step 3: Human Directs**
- "What's the next phase?"
- You initiate next phase
- Agent doesn't autonomously continue

---

## Task Structure for Video Generation

Your video generation workflow breaks into **4 phases with 12 atomic tasks**. Here's the breakdown:

### Phase 1: Setup & Verification (3 tasks, 30-45 minutes)

These tasks verify your environment is ready BEFORE attempting browser automation.

**Task 1.1: Verify Playwright MCP Configuration**
- **Duration**: 15 minutes
- **Depends on**: Nothing
- **What to do**: Check that Playwright MCP is installed and configured in Claude Desktop
- **Acceptance**: "Claude Desktop config file contains Playwright MCP with correct server path"
- **Output**: Verified `.claude/claude-config.json` file or console confirmation

**Task 1.2: Create Session Storage Directory**
- **Duration**: 10 minutes
- **Depends on**: Task 1.1
- **What to do**: Create `.session-storage/` directory for persisting browser sessions
- **Acceptance**: "Directory exists at correct path with 0755 permissions"
- **Output**: Directory visible in file explorer, path confirmed

**Task 1.3: Verify Gemini.google.com Accessibility**
- **Duration**: 15 minutes
- **Depends on**: Task 1.2
- **What to do**: Open Gemini in regular browser to confirm it's accessible from your network
- **Acceptance**: "Gemini loads fully with no access restrictions or geoblocking"
- **Output**: Screenshot showing Gemini.google.com homepage loaded

### Phase 2: Browser Session Setup (2 tasks, 30-45 minutes)

These tasks establish and verify a persistent browser session using Playwright MCP.

**Task 2.1: Execute First-Run Login Workflow**
- **Duration**: 20 minutes
- **Depends on**: Task 1.3
- **What to do**: Use Playwright MCP to navigate to Gemini and login (manually enter credentials)
- **Steps**:
  1. Playwright navigates to `https://gemini.google.com/`
  2. You enter Google credentials when prompted
  3. Browser saves session cookies to `.session-storage/gemini-session.json`
- **Acceptance**: "Session file exists, contains valid cookies, file size > 1KB"
- **Output**: Session file at `.session-storage/gemini-session.json` with persisted login state

**Task 2.2: Verify Session Persistence**
- **Duration**: 15 minutes
- **Depends on**: Task 2.1
- **What to do**: Close browser, reopen with saved session, verify still logged in
- **Steps**:
  1. Close Playwright browser instance
  2. Reopen Playwright with saved session file
  3. Navigate to Gemini
  4. Verify "already logged in" state (no login form)
- **Acceptance**: "Browser loads Gemini WITHOUT requiring login; session reused successfully"
- **Output**: Confirmation log showing second session reused cookies

### Phase 3: Video Generation (4 tasks, 60-90 minutes)

These tasks execute the actual video generation using the prompt prepared from L06 research.

**Task 3.1: Structure and Finalize Video Generation Prompt**
- **Duration**: 15 minutes
- **Depends on**: Task 2.2
- **What to do**: Take the research from L06 and structure it into the video prompt
- **Acceptance**: "Prompt has 4+ explicit scenes with start/end times, style keywords, and constraint descriptions"
- **Output**: Complete prompt text ready to paste into Gemini chat
- **THE PROMPT** (from L06 research, FINAL VERSION):
  ```
  Create a 45-second product demo video showing:

  Scene 1 (0-10s): Modern SaaS dashboard with clean UI
  - Show dashboard interface with charts, metrics, user list
  - Fade in smoothly from black
  - Keep animations minimal, business-focused

  Scene 2 (10-20s): User clicks "Sign Up" button
  - Button highlight, form appears with smooth slide-in
  - Shows email field, password field, name field
  - Form is modern, minimal, professional design

  Scene 3 (20-35s): Form filling animation
  - Auto-fill animation for each field (very smooth, 0.5s per field)
  - Fields light up as they complete
  - Smooth transitions between fields, no jarring movements

  Scene 4 (35-45s): Success confirmation
  - Confetti animation (subtle, professional, not cartoonish)
  - "Account created!" message fades in
  - Text overlay: "Sign up in under 60 seconds" appears
  - Fade to black

  Overall Style:
  - Professional corporate aesthetic (SaaS marketing standard)
  - Minimal design language (Apple-like, not cluttered)
  - Smooth transitions (nothing abrupt, 3+ second minimum per scene)
  - Color palette: Modern blues, grays, whites (tech company standard)

  Video Specifications:
  - Format: MP4 with H.264 codec
  - Resolution: 1920x1080 (standard for marketing)
  - Duration: 45-50 seconds (strict)
  - Frame rate: 30fps (standard for web)
  - Audio: Subtle upbeat corporate background music (no voiceover)

  Important:
  - NO spoken narration (music only)
  - NO distracting animations or emoji
  - NO flashy effects (this is B2B marketing, not entertainment)
  - Content must match prompt exactly (validate before delivery)
  ```

**Task 3.2: Login with Persisted Session and Navigate**
- **Duration**: 15 minutes
- **Depends on**: Task 3.1
- **What to do**: Use Playwright MCP to open Gemini with saved session
- **Acceptance**: "Gemini.google.com loads and confirms logged-in state in URL/page structure"
- **Output**: Console logs showing successful session reuse, no login prompts

**Task 3.3: Submit Video Generation Prompt to Gemini**
- **Duration**: 20 minutes
- **Depends on**: Task 3.2
- **What to do**: Paste prompt from Task 3.1 into Gemini chat and submit
- **Steps**:
  1. Playwright inputs prompt text into chat field
  2. Playwright presses Enter/Submit button
  3. Gemini begins video generation (shows progress indicator)
- **Acceptance**: "Gemini displays 'Generating...' message; progress indicator visible"
- **Output**: Screenshot or console log showing prompt submitted and generation starting

**Task 3.4: Wait for Generation and Download Video**
- **Duration**: 20 minutes
- **Depends on**: Task 3.3
- **What to do**: Wait for Gemini to complete video generation, then download
- **Steps**:
  1. Wait 60-90 seconds for Gemini to generate video
  2. Verify "Download" button appears on page
  3. Click download button (Playwright can trigger or you can manually)
  4. Video file downloads to Downloads folder as `Gemini-Generated-Video-*.mp4`
- **Acceptance**: "Video file exists in Downloads folder, file size > 1MB"
- **Output**: Video file path confirmed: `~/Downloads/Gemini-Generated-Video-[timestamp].mp4`

### Phase 4: Validation & Finalization (3 tasks, 45-60 minutes)

These tasks verify the video meets quality standards before considering project complete.

**Task 4.1: Move Video to Project Directory**
- **Duration**: 10 minutes
- **Depends on**: Task 3.4
- **What to do**: Move downloaded video from Downloads to project output folder
- **Steps**:
  1. Locate downloaded file: `~/Downloads/Gemini-Generated-Video-*.mp4`
  2. Create `./output/` directory if needed
  3. Move file to `./output/demo-video.mp4`
- **Acceptance**: "File exists at `./output/demo-video.mp4`, original removed from Downloads"
- **Output**: Video file at expected path in project directory

**Task 4.2: Verify Video File Format and Properties**
- **Duration**: 15 minutes
- **Depends on**: Task 4.1
- **What to do**: Validate video codec, resolution, duration using ffprobe
- **Commands**:
  ```bash
  ffprobe -v error -select_streams v:0 \
    -show_entries stream=codec_name,width,height,duration \
    ./output/demo-video.mp4
  ```
- **Acceptance Criteria** (ALL must pass):
  - Codec: H.264 (required for web compatibility)
  - Resolution: 1920x1080 or similar (not tiny)
  - Duration: 45-60 seconds (matches prompt requirement)
  - File size: > 2MB (indicates full render, not truncated)
- **Output**: ffprobe output showing all properties meet requirements

**Task 4.3: Test Video Playback and Validate Content**
- **Duration**: 20 minutes
- **Depends on**: Task 4.2
- **What to do**: Play video and verify content matches the prompt
- **What to check**:
  1. Video plays without errors or artifacts
  2. Dashboard scene visible at start
  3. Sign-up button and form visible in Scene 2
  4. Form filling animation smooth (not jerky)
  5. Success confirmation with confetti in Scene 4
  6. Text overlay "Sign up in under 60 seconds" present
  7. Audio is corporate background music (no narration)
  8. Transitions are smooth (no abrupt cuts)
- **Acceptance**: "Playback successful; all content scenes present and match prompt intent"
- **Output**: Playback logs; confirmation that content matches specification

---

## Checkpoint Sequence

Here's how the workflow actually progresses with human checkpoints:

### Checkpoint 1: After Phase 1 (Setup)
```
AGENT: "Setup verification complete:
        ✓ Playwright MCP configured
        ✓ Session storage directory created
        ✓ Gemini accessible without restrictions

        Ready for Phase 2: Browser Session Setup"

YOU: "Review... MCP is working, directory created.
      Looks good! Committing setup phase."

YOU: (git commit)

YOU: "Proceed to Phase 2"
```

### Checkpoint 2: After Phase 2 (Session)
```
AGENT: "Session setup complete:
        ✓ First-run login successful
        ✓ Session file saved to .session-storage/gemini-session.json
        ✓ Session persistence verified (logged in after restart)

        Ready for Phase 3: Video Generation"

YOU: "Review session file... valid cookies present.
      Committing session phase."

YOU: (git commit)

YOU: "Proceed to Phase 3"
```

### Checkpoint 3: After Phase 3 (Generation)
```
AGENT: "Video generation complete:
        ✓ Prompt submitted to Gemini
        ✓ Generation completed (90 seconds)
        ✓ Video downloaded: ~/Downloads/Gemini-Generated-Video-*.mp4
        ✓ Moved to ./output/demo-video.mp4

        Ready for Phase 4: Validation"

YOU: "Review video in file explorer... 3.2MB file.
      Committing generation phase."

YOU: (git commit)

YOU: "Proceed to Phase 4"
```

### Checkpoint 4: After Phase 4 (Validation)
```
AGENT: "Validation complete:
        ✓ File format: MP4 H.264 codec
        ✓ Duration: 48 seconds (within 45-60s requirement)
        ✓ Resolution: 1920x1080 (matches spec)
        ✓ Content validated: All 4 scenes present
        ✓ Quality gates: All pass

        PROJECT COMPLETE: demo-video.mp4 ready for use"

YOU: "Review ffprobe output... all properties correct.
      Watch final video... excellent quality.
      Committing validation phase. Project complete!"

YOU: (final git commit)
```

---

## Task Dependency Graph

Here's how your tasks depend on each other:

```
Phase 1 (Setup): Sequential path (each depends on previous)
┌────────────────────────────────────────────────────────────┐
│  Task 1.1: Verify Playwright MCP                           │
│      ↓                                                      │
│  Task 1.2: Create Session Directory                        │
│      ↓                                                      │
│  Task 1.3: Verify Gemini Access                            │
│      ↓ [CHECKPOINT 1]                                      │
└────────────────────────────────────────────────────────────┘

Phase 2 (Session): Sequential path (each depends on Phase 1)
┌────────────────────────────────────────────────────────────┐
│  Task 2.1: First-Run Login (depends on 1.3)                │
│      ↓                                                      │
│  Task 2.2: Verify Session Persistence                      │
│      ↓ [CHECKPOINT 2]                                      │
└────────────────────────────────────────────────────────────┘

Phase 3 (Generation): Linear path (each depends on previous)
┌────────────────────────────────────────────────────────────┐
│  Task 3.1: Structure Video Prompt (depends on 2.2)         │
│      ↓                                                      │
│  Task 3.2: Login with Session (depends on 3.1)             │
│      ↓                                                      │
│  Task 3.3: Submit Prompt to Gemini (depends on 3.2)        │
│      ↓                                                      │
│  Task 3.4: Wait and Download Video (depends on 3.3)        │
│      ↓ [CHECKPOINT 3]                                      │
└────────────────────────────────────────────────────────────┘

Phase 4 (Validation): Linear path (each depends on previous)
┌────────────────────────────────────────────────────────────┐
│  Task 4.1: Move Video to Output (depends on 3.4)           │
│      ↓                                                      │
│  Task 4.2: Verify File Properties (depends on 4.1)         │
│      ↓                                                      │
│  Task 4.3: Test Playback & Content (depends on 4.2)        │
│      ↓ [CHECKPOINT 4 - PROJECT COMPLETE]                   │
└────────────────────────────────────────────────────────────┘

Legend: Each task must complete before next starts (strict dependency)
        Checkpoints occur after each phase group
```

---

## Lineage Traceability

Can you trace a task back to specification? Try this one:

```
Specification: "Generate video demonstrating product demo workflow"
  ↓
Plan: "Phase 3: Video Generation - Submit refined prompt to Gemini,
       capture generated video, validate against quality criteria"
  ↓
Task 3.1: "Structure and Finalize Video Generation Prompt"
  ↓
Acceptance Criterion: "Prompt has 4+ explicit scenes with start/end times,
                       style keywords, and constraint descriptions"
```

If you can trace this lineage for each task, your task breakdown is well-connected to your specification.

---

## Common Mistakes

### Mistake 1: Tasks Too Large (45+ Minutes)

**The Error**: "Task: Complete entire video generation (2+ hours)"

**Why It's Wrong**: Large tasks hide complexity, delay feedback, and make checkpoints meaningless. You can't validate progress until the entire video generation completes.

**The Fix**: Break into atomic units (15-30 minutes each):
- ❌ Large: "Generate and validate video"
- ✅ Atomic: "Structure prompt" (15 min), "Submit to Gemini" (20 min), "Download" (20 min), "Validate format" (15 min)

### Mistake 2: Combining Manual and Automated Steps

**The Error**: Task includes "login, wait for generation, download, validate" all as one task

**Why It's Wrong**: If Gemini generation takes 90 seconds and you need to intervene (quality issue, timeout), you can't easily restart from the right point. Mixing manual + automated confuses where failures occur.

**The Fix**: Separate manual steps from automated:
- Task 3.2: "Login with persisted session" (automation)
- Task 3.3: "Submit prompt to Gemini" (automation)
- **CHECKPOINT**: Human reviews before continuing
- Task 3.4: "Wait for generation and download" (you watch, system generates)

### Mistake 3: Vague Acceptance Criteria

**The Error**: "Task: Setup is complete" (what does "complete" mean?)

**Why It's Wrong**: You won't know if the task is done or if there's a hidden failure.

**The Fix**: Make acceptance criteria specific and testable:
- ✅ "Session file exists at `.session-storage/gemini-session.json` AND file size > 1KB AND contains valid cookie tokens"
- ✅ "Video plays without errors AND all 4 scenes visible AND duration is 45-60 seconds"

---

## Try With AI

Ready to validate your task breakdown and prepare for implementation? Test your understanding:

**Explore Task Atomicity:**
> "I've broken video generation into 12 tasks across 4 phases. Review my task list: (1) Is each task atomic (does ONE thing with ONE acceptance criterion)? (2) Are they sized right (15-30 minutes, not hours or minutes)? (3) Can each be reviewed independently? (4) Identify any tasks that should be split further or combined. (5) Which tasks would you add or remove?"

**Practice Checkpoint Validation:**
> "Walk me through the checkpoint pattern for my video generation workflow. For each checkpoint (after Phases 1, 2, 3, 4): (1) What should I review for? (2) What makes a 'good' output at this checkpoint? (3) What issues could arise that I should catch before continuing? (4) Create a checklist I can use at each checkpoint to decide 'ready to proceed'."

**Analyze Dependencies:**
> "Examine my task dependencies: (1) Are they logically correct? (2) Which tasks could theoretically run in parallel? (3) What's the critical path (minimum sequence to completion)? (4) If one task failed (e.g., video generation timeout), which downstream tasks would be affected? (5) How would I recover and restart?"

**Test Prompt Quality:**
> "Review my video generation prompt (Task 3.1): (1) Does it have explicit scene structure with start/end times? (2) Are style keywords clear enough for Gemini to understand? (3) Are constraints explicit (resolution, duration, codec)? (4) What ambiguities exist that Gemini might interpret differently? (5) How would you improve this prompt to increase likelihood of first-attempt success?"

---
