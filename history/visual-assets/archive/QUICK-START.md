# Quick Start - Image Generation

**Status**: Ready to execute
**Total images**: 4
**Estimated time**: 2-4 hours

---

## One-Line Instruction

Copy and paste this into your other session:

```
Use the image-generator skill to generate all visual assets for Chapter 1 lessons with embedded IMAGE GENERATION PROMPT comments. There are 4 prompts total across 3 lesson files: lesson 02 (2 images), lesson 04 (1 image), and lesson 06 (1 image). All prompts use correct Polar Night brand colors (Deep Navy #001f3f). Read the full instructions in history/visual-assets/INSTRUCTIONS-FOR-IMAGE-GENERATION.md if needed.
```

---

## What Will Happen

The `image-generator` skill will automatically:
1. Find all 4 embedded prompts in lesson markdown files
2. Open Gemini.google.com via Playwright MCP (you'll need to log in once)
3. Generate each image (one session per image, verify before downloading)
4. Save to `book-source/static/img/part-1/chapter-1/`
5. Update markdown files (replace HTML comments with `![](path)`)
6. Create completion report

---

## Expected Output

**4 PNG files saved:**
- `developer-economy-calculation-breakdown.png` (16:9)
- `technology-adoption-speed-acceleration.png` (16:9)
- `development-lifecycle-transformation-diagram.png` (16:9)
- `ai-coding-tools-evolution-timeline.png` (9:16 vertical)

**Completion report:**
- `history/visual-assets/chapter-1-image-generation-report.md`

---

## If You Need Help

Read the detailed instructions:
- `history/visual-assets/INSTRUCTIONS-FOR-IMAGE-GENERATION.md`

Or check the skill definition:
- `.claude/skills/image-generator/SKILL.md`

---

**Ready? Copy the instruction above and paste it into your other session!** 🚀
