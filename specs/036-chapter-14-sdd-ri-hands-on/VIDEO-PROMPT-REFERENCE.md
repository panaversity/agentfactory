# Video Prompt Reference — Task 3.1 Deliverable

**Status**: LESSON 07 OUTPUT
**Location**: Chapter 14, Lesson 07, Task 3.1
**Purpose**: Ready-to-use prompt for Gemini.google.com video generation

---

## PRODUCTION PROMPT — Copy and Paste Ready

Use this prompt exactly as-is when submitting to Gemini for video generation.

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

---

## Prompt Engineering Breakdown

This prompt works effectively because of its structure:

### 1. Scene-by-Scene Structure
**Why it works**: Gemini understands explicit scene descriptions better than abstract requests.
- Each scene has duration (0-10s, 10-20s, etc.)
- Each scene has visual elements (dashboard, button, form, etc.)
- Transitions are specified (fade in, slide-in, fade to black)

### 2. Style Keywords
**Why it works**: Gemini responds to specific style descriptors.
- "SaaS marketing standard" (signals: professional, not entertainment)
- "Minimal design language" (signals: Apple-like, not cluttered)
- "Professional corporate aesthetic" (signals: B2B, not B2C)
- "Smooth transitions" (signals: quality over flashiness)

### 3. Technical Constraints
**Why it works**: Explicit constraints prevent quality degradation.
- Format: MP4 with H.264 codec (standard for web distribution)
- Resolution: 1920x1080 (not too small, not wasting bandwidth)
- Duration: 45-50 seconds (strict, not "around 45 seconds")
- Frame rate: 30fps (standard for web video, not theatrical)
- Audio: specific (corporate background, no voiceover)

### 4. Explicit Prohibitions
**Why it works**: Tells Gemini what NOT to do, preventing common mistakes.
- NO spoken narration (music only)
- NO distracting animations (prevents emoji, sparkles, over-animation)
- NO flashy effects (keeps focus on content, not effects)
- NO cartoonish elements (keeps professional tone)

---

## When to Use This Prompt

### Use This Prompt If:
- You're creating product demo for SaaS product
- Target audience is business decision-makers (not consumers)
- Goal is to show user workflow (signup, onboarding, etc.)
- Budget is limited (can't afford professional production)

### Modify Prompt If:
- Your product is mobile-first → Change "dashboard" to "mobile app screen"
- Your product is API → Change "dashboard" to "code editor with API calls"
- Your product is consumer-facing → Remove "B2B marketing standard", add "modern, energetic, youthful"
- Your demo is longer → Adjust duration (e.g., "120-second" instead of "45-second")

---

## Expected Output

When Gemini processes this prompt, it should generate:

### Quality Indicators (You'll Verify in Task 4.3)
- [ ] Dashboard visible with real-looking UI elements
- [ ] Sign-up button and form visible in Scene 2
- [ ] Form filling animation smooth (no stuttering)
- [ ] Success confirmation with confetti (not cartoonish)
- [ ] Text overlay readable and appropriately timed
- [ ] Overall duration 45-50 seconds (not too long, not too short)
- [ ] Audio is background music (no voiceover, no silence)
- [ ] Transitions smooth (no abrupt cuts)
- [ ] Color palette blues/grays/whites (professional, not bright)

### What You'll Get
```
~/Downloads/Gemini-Generated-Video-[timestamp].mp4
```

**File size**: Usually 2-5MB depending on quality and complexity
**Duration**: 45-50 seconds (matches constraint)
**Format**: MP4 H.264 codec (web-ready, no re-encoding needed)
**Location after Task 4.1**: `./output/demo-video.mp4`

---

## How This Prompt Was Developed

This prompt represents applied learning from Chapter 13 (SDD-RI Theory):

1. **Specification Phase (L04)**: Defined what success looks like for product demo video
2. **Research Phase (L06)**: Discovered what prompts work (scene structure, timing, style keywords)
3. **Task Phase (THIS LESSON)**: Structured research into executable prompt
4. **Implementation Phase (L08)**: You'll execute this prompt with Playwright MCP + Gemini

---

## Troubleshooting

### If Video Quality Is Poor

**Check these aspects of prompt**:
1. **Scene clarity**: Are scenes described with specific visual elements?
2. **Timing**: Are durations explicit? (Not "quick", but "0-10s", "10-20s")
3. **Style keywords**: Are they understood by Gemini? (Test: "professional corporate" vs "fancy business")
4. **Constraints**: Are prohibitions clear? (NO vs "minimize" or "reduce")

**If yes to all, try rephrasing**:
```
Instead of "professional corporate aesthetic":
Try: "clean, simple, business-friendly design"

Instead of "minimal design language":
Try: "Apple-like, very clean, no extra elements"

Instead of "smooth transitions":
Try: "slow, deliberate movements between scenes (3+ seconds each)"
```

### If Gemini Generates Wrong Scene Structure

**Issue**: Gemini ignored scene timing and merged scenes
**Solution**: Add explicit scene numbering in prompt:
```
SCENE 1 (0-10 seconds): [description]
- [visual element 1]
- [visual element 2]

SCENE 2 (10-20 seconds): [description]
- [visual element 1]
- [visual element 2]
```

### If Video Duration Is Wrong

**Issue**: Generated video is too short or too long
**Solution**: Specify minimum scene duration:
```
Each scene must be at least 3-4 seconds of content (no fast cuts).
Total video duration must be 45-50 seconds (strict requirement).
```

---

## Learning Outcome

By working with this prompt, you understand:

1. **Prompt Engineering for AI Video**: Scene-by-scene structure, explicit constraints, style keywords
2. **Product Marketing**: What makes an effective product demo (workflow, not features)
3. **Quality Standards**: How to specify video technical requirements (codec, resolution, frame rate)
4. **Spec-Driven Development**: How clear specifications produce better AI outputs

---

## Next Steps

1. **Complete Tasks 1-3** (Setup, Session, Generation tasks)
2. **Use this prompt in Task 3.1** (Structure and Finalize)
3. **Submit to Gemini in Task 3.3** (Paste exact prompt text)
4. **Validate output in Task 4.3** (Verify all quality indicators above)
5. **Learn from results** (What worked? What would you change?)

---

**Ready to generate your first AI video!**

Copy this prompt exactly, follow Task 3 steps in Lesson 07, and watch Gemini create your product demo video.
