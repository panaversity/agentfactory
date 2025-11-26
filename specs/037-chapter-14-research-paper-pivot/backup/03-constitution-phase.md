---
title: "Constitution Phase - Project-Wide Rules"
chapter: 14
lesson: 3
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Creating Project Constitution"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write project Constitution defining video output quality, Gemini usage constraints, and testing requirements"

  - name: "Distinguishing Global Rules from Feature Requirements"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain difference between Constitution (applies to ALL video features) and Specification (applies to ONE video generation task)"

  - name: "Understanding Cascade Starting Point"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student recognizes that Constitution quality determines all downstream video generation quality"

  - name: "Defining Quality Standards for AI-Generated Content"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student can define testable quality criteria for video output (format, codec, duration, playback validation)"

  - name: "Technical Constraint Documentation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student documents Gemini free tier constraints, Playwright session requirements, and recovery patterns"

learning_objectives:
  - objective: "Write a project Constitution defining output quality, Gemini constraints, and testing requirements for video generation project"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Constitution document review against template completeness, testability, and video-domain relevance"

  - objective: "Explain why Constitution is created once per project, not per video generation task"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Written or verbal explanation of Constitution scope across video, thumbnail, metadata features"

  - objective: "Understand how Constitution quality cascades through all downstream phases"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Identification of how Constitution rules affect video specifications and implementation plans"

cognitive_load:
  new_concepts: 9
  assessment: "9 new concepts (Constitution role, Global vs feature rules, Cascade starting point, Video quality standards, Gemini constraints, Playwright requirements, Testing approach, Retry patterns, Git workflow integration) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Write Constitution for 2-3 different video project types (product demo, tutorial, social media clips); compare how Gemini, output format, and quality standards differ by project type"
  remedial_for_struggling: "Use provided Constitution template; fill in only essential sections (video output, Gemini constraints, error recovery, testing) before moving to Lesson 4"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/chapter-14-video-pivot/spec.md"
created: "2025-11-25"
last_modified: "2025-11-25"
git_author: "Claude Code"
workflow: "manual-implementation"
version: "1.0.0"
---

# Constitution Phase - Project-Wide Rules

Welcome to the third lesson of hands-on SDD-RI development. You've installed the Spec-Kit Plus framework and configured your AI tool for Playwright automation. Now it's time to establish the **foundational rules** that will guide every part of your video generation project.

The Constitution is Spec-Kit Plus's answer to a critical question: **What standards apply to every piece of work you do?** Not just for this video feature, but for all features. Not just for this demo, but for the life of the project.

Imagine you're working with your AI assistant to generate product demo videos. Before you start, you need to agree on rules so nothing breaks:

*   What if you want all videos to be **MP4 format with H.264 codec**, but your assistant generates **WebM format**?
*   What if you decide videos must **download successfully** within **60 seconds**, but your assistant times out at **90 seconds**?

That would be a mess!

The **Constitution** is your team's **rulebook** for video generation. It defines the standards that both you and your AI orchestrator **MUST** follow. It makes sure every video, every upload, every automation workflow follows the same quality standards.

---

## What Is a Constitution?

Before writing one, let's understand what a Constitution actually is and why it matters for video generation.

### Constitution: Global Rules, One Per Project

A **Constitution** is a document that defines **immutable standards** applying to **all video-related features** in a project. It's distinct from a **Specification**, which applies to **one feature**.

**Constitution applies to**:
- Video output standards (format, codec, resolution, duration limits)
- Gemini.google.com usage constraints (free tier limits, session handling, timeout policy)
- Browser automation patterns (Playwright session persistence, error recovery, retry logic)
- Testing and validation (video playback verification, file format validation, automated testing)
- Quality gates (download success criteria, format verification, duration validation)
- Error handling strategies (retry policies, timeout handling, session recovery)

**Specification applies to**:
- Specific video feature requirements (product demo script, background music, text overlay specifications)
- User stories for that video feature
- Acceptance criteria for that specific video
- Edge cases specific to that video type

**Example**:

```
CONSTITUTION (applies to ALL video features):
  ✅ "All video outputs must be MP4 format with H.264 codec"
  ✅ "All videos must be downloadable within 60 seconds"
  ✅ "Gemini timeouts retry up to 3 times before failing"
  ✅ "All video downloads must be validated for format and playability"

SPECIFICATION (applies only to PRODUCT DEMO video):
  ✅ "Demo video shows sign-up workflow in 45 seconds"
  ✅ "Background music: royalty-free, 10 seconds, fades at end"
  ✅ "Text overlay displays company logo (top-right corner)"
  ✅ "Video background: clean desk environment with neutral colors"
```

### Why Constitution Matters: The Cascade

The Constitution is the **starting point of the cascade**:

```
Clear Constitution
    ↓
(ensures that every spec follows video quality standards)
    ↓
Clear Specification
    ↓
(ensures that planning accounts for Gemini constraints)
    ↓
Clear Plan
    ↓
(ensures that tasks handle Playwright automation correctly)
    ↓
Clear Tasks
    ↓
(enables AI to generate reliable video automation code)
    ↓
Working Video Generation System
```

**Weak Constitution** produces:
- Specs that don't specify video format (leading to incompatible outputs)
- Plans that don't account for Gemini free tier timeouts
- Code that lacks error recovery when sessions fail
- Videos that can't be validated for quality after generation
- Integration issues because output standards weren't enforced upstream

**Strong Constitution** produces:
- Specs that automatically include output format requirements
- Plans with Gemini timeout handling built in
- Code with robust session recovery and retry logic
- Automated validation that verifies video format and playability
- Integration that works because standards were clear from the start

### Constitution is One-Time, Video Work is Repetitive

This is crucial: You write the Constitution **once per project**. Then, for each video feature, you:

1. Write a specification (addressing this video feature only)
2. Generate a plan
3. Generate tasks
4. Implement the video generation code

But you never rewrite the Constitution for each video. It's the foundation everything builds on.

**Best Practice Pattern**:

```
1. Initialize project
2. Write Constitution
3. Commit Constitution to git
4. FOR EACH VIDEO FEATURE:
   - Run /sp.specify (new specification)
   - Run /sp.clarify (refine specification)
   - Run /sp.plan (new plan for this video)
   - Run /sp.tasks (new tasks for this video)
   - Run /sp.implement (new video generation for this video)
   - Commit video feature to git
```

#### 💬 AI Colearning Prompt
> "Why does the Constitution cascade its quality to all downstream phases? What would happen if I wrote a vague Constitution about video output (like 'good quality videos') but tried to write a precise specification for a product demo video?"

---

## Reading Existing Constitutions (15 minutes)

Before writing your own, let's look at the base Constitution file. Open:

```bash
.specify/memory/constitution.md
```

Remember the Goal: document the non-negotiable principles that every video spec, plan, and task must honor.

**The Key Insight**: Constitutions are project-specific. Your video generation Constitution focuses on Gemini constraints, output formats, and browser automation—irrelevant for a calculator or data pipeline project.

#### 🎓 Expert Insight
> In AI-native development, the Constitution isn't bureaucracy—it's leverage. Write it once with clear, testable standards ("all videos must be MP4 with H.264 codec and download within 60 seconds"), and every AI-generated spec, plan, and implementation automatically inherits those standards. Vague Constitutions produce vague downstream work. Precise Constitutions produce precise downstream work. The 30 minutes you invest here saves hours of rework later when you discover your videos are in the wrong format or fail to download reliably.

---

## Part B: Writing Your Video Generation Constitution

Now let's write YOUR Constitution for the video generation project.

### Step 1: Create the Constitution

In your project directory:

1. Start an agent chat and type `/sp.constitution`.
2. Explain your video project requirements:
```
/sp.constitution

Project principles and standards:
- Generate reliable, downloadable videos from Gemini.google.com
- Use Playwright MCP for browser automation and session management
- Ensure all videos pass quality validation before storage
- Document all AI interactions with clear error recovery
- Follow production-ready patterns for automation

Technical stack:
- Gemini.google.com (free tier, no API key required)
- Playwright MCP for browser control and session persistence
- Node.js environment for automation
- Local file storage with validation

Quality requirements:
- All videos must be MP4 format with H.264 codec
- Video duration: 45-60 seconds (within Gemini free tier limits)
- All videos must validate successfully after download (format check, playability test)
- Gemini timeouts retry up to 3 times before failing
- Session recovery for interrupted Playwright operations
- All automation operations logged for debugging
```

3. Agent Does:
  - Creates comprehensive constitution file
  - Sets up video output standards and Gemini constraints
  - Defines browser automation patterns and session recovery
  - Establishes quality gates and validation requirements

### Step 2: Improve your Constitution

Think about what "good video generation" means for this project:

```markdown
Update .specify/memory/constitution.md to improve Video Output Standards

## Video Output Standards
- All videos must be MP4 format with H.264 codec
  - Example: `video_demo_2025-11-25.mp4` (not WebM, not MOV, not AVI)
- Resolution: 1920x1080 preferred, 1280x720 acceptable
- Frame rate: 30 FPS (standard for web videos)
- File size: under 50MB for reliable YouTube upload
- Duration: 45-60 seconds (respects Gemini free tier timeout)

## Gemini.google.com Constraints
- Free tier only (no API key, browser-based)
- Maximum generation timeout: 90 seconds wall-clock time
- Session persistence required (maintain Google authentication across retries)
- No concurrent video generations (single-session model)
- Rate limiting: 1 video per 5 minutes (conservative, prevents blocks)

## Browser Automation (Playwright MCP)
- Session persistence: maintain browser state across operations
- Timeout handling: retry failed downloads up to 3 times
- Error recovery: detect and recover from network interruptions
- Logging: log all automation steps for debugging

## Quality Validation
- Video download verification: confirm file exists and is readable
- Format validation: verify MP4 codec and container
- Playability test: attempt 5-second playback using system video player
- Metadata check: confirm duration is 45-60 seconds as specified
```

**Try writing 4-5 quality standards for your video project. Think about**:
- What format and constraints will ensure YouTube accepts your videos?
- What would prevent videos from uploading reliably?
- What Gemini limits might break your automation?
- How will you know if video generation succeeded?

### Step 3: Review and Complete Your Constitution

**Your Prompt:**

```
Show me the generated constitution file and explain what it contains.
```

**Agent Does:**

- Displays the constitution content
- Explains each section:
  - **Video Output Standards** - Format, codec, resolution, duration constraints
  - **Gemini Constraints** - Free tier limits, timeout policy, session requirements
  - **Browser Automation Patterns** - Session handling, error recovery, retry logic
  - **Quality Validation** - How videos are tested after generation
  - **Error Handling Strategy** - Retry policies and session recovery patterns

---

## Part C: Commit Constitution to Git (15 minutes)

Here's a critical best practice: **Always commit the Constitution before starting feature work.**

### Why Commit First?

1. **Immutability**: Constitution is foundational; committing it signals "this is our standard for video generation"
2. **Clarity**: Everyone (including your AI orchestrator) sees the Constitution as the baseline for all video features
3. **Traceability**: Git history shows when and why Constitution was created and updated
4. **Reversibility**: If you need to, you can revert to a previous Constitution (rarely happens, but important)

### Commit Steps

**Your Prompt**

Use the agent to commit and open a PR for the constitution:

```
/sp.git.commit_pr Commit and push the constitution along with current work.
```

**Agent Does:**

- Create a conventional commit for the constitution and push to a new feature branch
- Create a draft PR (or share the compare URL if `gh` auth is missing)


The Constitution is now **the foundation** for all your video generation work. Every specification you write, every plan you generate, every task you break down—they all work within the Constitution's constraints.

#### 🤝 Practice Exercise

> **Ask your AI**: "I've created my video generation Constitution with standards for MP4 format, Gemini timeout handling, and video playback validation. Can you review `.specify/memory/constitution.md` and tell me: (1) Are my quality standards testable or vague? (2) Did I miss any critical standards for reliable video generation? Then explain how these standards will cascade to my video specifications and plans."

**Expected Outcome**: Your AI should identify whether standards like "all videos must be MP4 with H.264 codec and download within 60 seconds" are testable (✅) versus vague phrases like "good video quality" (❌), suggest any missing video-specific standards (like Gemini timeout retry logic), and explain how the Constitution influences video specification-writing.

---

## Common Mistakes

### Mistake 1: Copying Constitution from Another Project Without Customization

**The Error**: "I'll just use a Python calculator Constitution for my video project."

**Why It's Wrong**: Constitutions are project-specific. A calculator Constitution focuses on type hints and error handling—irrelevant for video generation. A video Constitution must address Gemini constraints, video format standards, and browser automation patterns.

**The Fix**: Read example Constitutions for structure, but write rules specific to YOUR project needs. Your Constitution should be unrecognizable to someone building a calculator—it's that different.

### Mistake 2: Vague Video Quality Standards

**The Error**: "Videos must be good quality" or "Download should work reliably"

**Why It's Wrong**: "Good" and "reliably" are subjective. No one can verify these during code review or automated testing.

**The Fix**: Use testable criteria:
- ❌ Vague: "Videos should be good quality"
- ✅ Testable: "All videos MP4 format with H.264 codec; duration 45-60 seconds; download completes within 60 seconds; playback validates successfully"

### Mistake 3: Ignoring Gemini Free Tier Constraints

**The Error**: "Constitution doesn't mention Gemini timeout or session limits—we'll figure it out during implementation"

**Why It's Wrong**: Specs, plans, and implementations written downstream won't account for these constraints. You'll discover them too late, when code fails mysteriously.

**The Fix**: Document Gemini constraints explicitly:
- ✅ "Gemini free tier maximum timeout: 90 seconds"
- ✅ "Session recovery: retry up to 3 times on timeout"
- ✅ "Rate limiting: 1 video per 5 minutes to prevent blocks"

---

## Try With AI

Ready to validate your Constitution and understand how quality rules cascade through your video generation project? Test your setup:

**🔍 Explore Cascade Effect:**
> "I wrote a Constitution for my video generation project defining MP4 format, Gemini timeout handling, and playback validation. Explain how these rules cascade into downstream work: If my Constitution requires 'all videos must download within 60 seconds and pass playback validation', how does that constraint affect the Specification phase? The Planning phase? The Implementation phase? Show me the cascade with a concrete example for a product demo video."

**🎯 Practice Constitution Review:**
> "Review my Constitution at `.specify/memory/constitution.md`. Check for: (1) Are all rules testable and specific (not vague)? (2) Did I cover essential categories (video output standards, Gemini constraints, browser automation, validation, error handling)? (3) Are any rules too strict or unrealistic for Gemini free tier? Suggest 2-3 improvements to make it clearer and more practical for video generation."

**🧪 Test Specification Alignment:**
> "Imagine I'm writing a specification for a product demo video. Based on my Constitution rules, what constraints MUST the video spec include? Walk through: output format requirements (MP4/H.264), Gemini timeout handling (retry logic), download validation requirements (format/playability checks), and quality validation steps. This tests if my Constitution is specific enough to guide video specifications."

**🚀 Apply to Your Project:**
> "I'm planning to generate [describe your video type: product demo / tutorial / social media clip]. Help me draft a Constitution tailored to my video project. What output standards, Gemini constraint documentation, Playwright automation patterns, and validation requirements should I define? Compare how Constitution rules differ between my video type and a generic example."

---
