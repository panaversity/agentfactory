# Instructions for Image Generation Session

**Date**: 2025-01-12
**Status**: Ready for execution
**Estimated time**: 2-4 hours (including iterations)

---

## Context

You are about to generate 4 professional educational infographics for Chapter 1 of the AI-Native Software Development book. All visual prompts have been:

- ✅ Embedded as HTML comments in lesson markdown files
- ✅ Validated for pedagogical value (100% PASS rate)
- ✅ Corrected to use Polar Night brand colors (Deep Navy #001f3f)
- ✅ Designed with complete specifications ready for Gemini image generation

---

## Your Task

**Use the `image-generator` skill to generate 4 images from embedded prompts in Chapter 1 lessons.**

### Simple Invocation

```
Use the image-generator skill to generate all visual assets for Chapter 1 lessons with embedded IMAGE GENERATION PROMPT comments. There are 4 prompts total across 3 lesson files.
```

---

## What the image-generator Skill Will Do

The skill will automatically:

1. **Scan lesson files** for `<!-- VISUAL ASSET N: ... IMAGE GENERATION PROMPT: ... -->` comments
2. **Initialize browser** (Playwright MCP → Gemini.google.com)
3. **For each image** (one session per image):
   - Generate image from embedded prompt
   - **Verify IMMEDIATELY** using screenshot (check spelling, layout, colors)
   - Iterate if needed (letter-by-letter spelling for mistakes, try different approach after 3 failures)
   - Download to correct location when quality passes
   - Apply teaching effectiveness validation (< 5 second test)
4. **Update markdown** files (replace HTML comments with `![alt](path)` syntax)
5. **Create completion report** in `history/visual-assets/`

---

## Files Containing Prompts (4 Total)

### 1. Lesson 02 - Two Prompts

**File**: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/02-three-trillion-developer-economy.md`

**Visual 1** (lines 65-152):

- **Title**: Developer Economy Calculation Flow
- **Type**: Horizontal flow diagram (3 cards with × and = operators)
- **Filename**: `developer-economy-calculation-breakdown.png`
- **Dimensions**: 16:9 (1792x1024px)

**Visual 2** (lines 222-344):

- **Title**: Technology Adoption Speed Acceleration Timeline
- **Type**: Horizontal descending staircase bars (5 tech waves)
- **Filename**: `technology-adoption-speed-acceleration.png`
- **Dimensions**: 16:9 (1792x1024px)

---

### 2. Lesson 04 - One Prompt

**File**: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/04-development-lifecycle-transition.md`

**Visual 1** (lines 79-240):

- **Title**: Development Lifecycle Transformation Dual-Track
- **Type**: Parallel process flows (6 phases each, traditional vs AI)
- **Filename**: `development-lifecycle-transformation-diagram.png`
- **Dimensions**: 16:9 (1792x1024px)

---

### 3. Lesson 06 - One Prompt

**File**: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/06-autonomous-agent-era.md`

**Visual 1** (lines 115-247):

- **Title**: AI Coding Tools Evolution Timeline
- **Type**: Vertical timeline with 4 generation cards and autonomy meter
- **Filename**: `ai-coding-tools-evolution-timeline.png`
- **Dimensions**: 9:16 vertical (1024x1792px)

---

## Critical Quality Standards

### Brand Colors (Polar Night Theme)

**These are CORRECTED - all prompts use these colors:**

- **Primary accent**: Deep Navy (#001f3f)
- **Dark text**: Dark Charcoal (#111111)
- **Medium text**: Medium Gray (#aaaaaa)
- **Light backgrounds**: Light Gray (#dddddd)
- **White**: Pure White (#FFFFFF)

### Spelling Accuracy (99% Quality Mindset)

**Common issues to watch for:**

- Company names: "Combinator" (not "Cymbinator"), "GitHub", "Y Combinator"
- Technical terms: "Autonomous", "Infrastructure", "Implementation"
- **Solution**: If Gemini misspells, use letter-by-letter spelling in refinement prompt

### Layout Precision

**If proportions/bars fail after 3 tries:**

- Try DIFFERENT approach (text-based "VS" comparison instead of bars)
- Use explicit ratio labels ("180x faster") instead of relying on visual proportions

### Teaching Effectiveness Validation

**After generating each image, ask yourself:**

1. Can I understand the key concept in < 5 seconds?
2. Does this TEACH a pattern, or just SHOW data?
3. Can I state the teaching goal in one sentence?
4. Does this reduce cognitive load?

**If answer is NO to any**: Flag for redesign or removal (document in completion report)

---

## Expected Output

### Images Saved To:

`book-source/static/img/part-1/chapter-1/`

**4 files:**

- `developer-economy-calculation-breakdown.png`
- `technology-adoption-speed-acceleration.png`
- `development-lifecycle-transformation-diagram.png`
- `ai-coding-tools-evolution-timeline.png`

### Markdown Files Updated:

**HTML comment prompts replaced with:**

```markdown
![Alt text here](/img/part-1/chapter-1/filename.png)
```

### Completion Report Created:

`history/visual-assets/chapter-1-image-generation-report.md`

**Should include for each image:**

- Total iterations needed
- Issues encountered and solutions applied
- Prompting strategies that worked
- Teaching effectiveness assessment (PASS/CONDITIONAL/FAIL)
- Quality rating (Excellent / Good / Acceptable)

---

## Workflow Reminders (From image-generator Skill)

### One Session Per Image

- Generate → Review → Iterate → Download → **NEW SESSION**
- Click "New chat" button before starting next image
- Prevents context contamination and makes review cleaner

### Verify BEFORE Downloading

- Use `browser_snapshot` or `browser_take_screenshot` to VIEW image
- Check spelling, layout, colors IMMEDIATELY
- Never download until quality passes

### Iteration Strategy

- **1-3 tries**: Refine prompt in same session
- **After 3 failures with same approach**: Try COMPLETELY DIFFERENT approach
- **After 4 iterations total**: Document issue and proceed (or remove visual)

---

## If You Encounter Issues

### Gemini Login Required

- Pause and ask user to log in (one-time authentication)
- Browser session will maintain login across all 4 images

### Spelling Errors

- Refine prompt with letter-by-letter spelling: "Combinator (C-O-M-B-I-N-A-T-O-R)"
- Use simpler/abbreviated forms: "YC W25" instead of full name

### Layout Failures

- After 3 tries with bars/proportions, switch to text-based design
- Use explicit labels instead of visual ratios

### Browser Issues

- Page not loading: Retry navigation, check network
- Tool not appearing: Refresh page, try new chat

---

## Success Criteria

✅ **All 4 images generated** and saved to correct directory
✅ **All images pass teaching effectiveness test** (< 5 second understanding)
✅ **All markdown files updated** (HTML comments replaced with image syntax)
✅ **Completion report created** with iteration notes and quality assessments
✅ **Images render correctly** in Docusaurus (optional: run dev server to verify)

---

## Estimated Timeline

| Task                            | Time          |
| ------------------------------- | ------------- |
| Browser setup + Gemini login    | 5 min         |
| Image 1 generation + iterations | 20-40 min     |
| Image 2 generation + iterations | 20-40 min     |
| Image 3 generation + iterations | 20-40 min     |
| Image 4 generation + iterations | 20-40 min     |
| Markdown updates                | 10 min        |
| Completion report               | 15 min        |
| **Total**                       | **2-4 hours** |

---

## Ready to Start?

**Invoke the skill with:**

```
Use the image-generator skill to generate all visual assets for Chapter 1 lessons with embedded IMAGE GENERATION PROMPT comments. There are 4 prompts total across 3 lesson files.
```

The skill will handle the rest automatically, following the workflow defined in `.claude/skills/image-generator/SKILL.md`.

---

## Reference Documents

- **Audit report**: `history/visual-assets/READY-FOR-IMAGE-GENERATION-v2.md`
- **Skill definition**: `.claude/skills/image-generator/SKILL.md`
- **Brand colors source**: `book-source/src/css/custom.css`

---

**Good luck! Remember: 99% quality mindset - Gemini CAN get it right, iterate until perfect.** 🚀
