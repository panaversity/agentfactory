# Chapter 2 Visual Assets - Resume Instructions

## CURRENT STATUS

**✅ COMPLETED:**

- All 8 image prompts embedded in lesson markdown files
- Audit report created: `history/visual-assets/chapter-2-audit-report.md`
- Image 1 generated & saved: `book-source/static/img/part-1/chapter-2/evidence-compared-2024-vs-2025.png` ✓
- Image 2 generated & saved: `book-source/static/img/part-1/chapter-2/ai-capability-breakthroughs-2025.png` ✓

**⏸️ REMAINING (6 images):**

- Image 3: Paradigm Shift Interface to Intent → `paradigm-shift-interface-to-intent.png`
- Image 4: Five Powers of AI Agents → `five-powers-ai-agents.png`
- Image 5: Team A vs B Timeline → `team-comparison-vibe-vs-spec-timeline.png`
- Image 6: Seven DORA Capabilities → `seven-dora-capabilities-ai-success.png`
- Image 7: Three-Layer Stack → `three-layer-ai-development-stack.png`
- Image 8: Stack Evolution → `stack-evolution-2024-to-2025.png`

**THEN:**

- Update all 8 markdown files to replace HTML comments with image references
- Create completion report

---

## WORKFLOW TO RESUME

### Step 1: Generate Remaining 6 Images

For each image (#3-8):

1. **Navigate to Gemini**: https://gemini.google.com/app
2. **Click**: "🍌 Create Image" button
3. **Find prompt**: Open corresponding lesson file, copy IMAGE GENERATION PROMPT from HTML comment
4. **Paste & Generate**: Submit prompt, wait ~30 seconds
5. **Download**: Click "Download full size image"
6. **Copy to location**:

   ```bash
   # Find most recent download
   ls -lt .playwright-mcp/Gemini-Generated-Image-*.png | head -1

   # Copy with correct filename
   cp .playwright-mcp/Gemini-Generated-Image-XXXXXX.png \
      book-source/static/img/part-1/chapter-2/{FILENAME}.png
   ```

### Lesson Files with Prompts

```
Image 3: apps/learn-app/docs/01-Introducing-AI-Driven-Development/02-ai-turning-point/02-user-interface-intent.md
         (VISUAL ASSET 1 around line 136)
         Filename: paradigm-shift-interface-to-intent.png

Image 4: Same file as Image 3
         (VISUAL ASSET 2 around line 308)
         Filename: five-powers-ai-agents.png

Image 5: apps/learn-app/docs/01-Introducing-AI-Driven-Development/02-ai-turning-point/03-development-patterns.md
         (VISUAL ASSET 1 around line 183)
         Filename: team-comparison-vibe-vs-spec-timeline.png

Image 6: apps/learn-app/docs/01-Introducing-AI-Driven-Development/02-ai-turning-point/04-dora-perspective.md
         (VISUAL ASSET 1 around line 140)
         Filename: seven-dora-capabilities-ai-success.png

Image 7: apps/learn-app/docs/01-Introducing-AI-Driven-Development/02-ai-turning-point/05-ai-coding-agents.md
         (VISUAL ASSET 1 around line 114)
         Filename: three-layer-ai-development-stack.png

Image 8: Same file as Image 7
         (VISUAL ASSET 2 around line 247)
         Filename: stack-evolution-2024-to-2025.png
```

---

### Step 2: Update All Markdown Files

After all 8 images are generated, replace HTML comments with image tags:

**Pattern**: Find `<!-- VISUAL ASSET N: ... -->` blocks, replace with:

```markdown
![{alt-text}](/img/part-1/chapter-2/{filename})
```

**Files to update:**

1. `01-the-inflection-point.md` (2 images)
2. `02-user-interface-intent.md` (2 images)
3. `03-development-patterns.md` (1 image)
4. `04-dora-perspective.md` (1 image)
5. `05-ai-coding-agents.md` (2 images)

---

### Step 3: Create Completion Report

Create: `history/visual-assets/chapter-2-visual-assets-report.md`

**Template:**

```markdown
# Chapter 2 Visual Assets - Completion Report

**Date**: {today}
**Status**: ✅ Complete
**Total Assets**: 8
**Total Iterations**: {count refinements if any}

## Generated Images

### Image 1: Evidence Compared 2024 vs 2025

- **Filename**: evidence-compared-2024-vs-2025.png
- **Lesson**: 01-the-inflection-point.md
- **Iterations**: 1 (first attempt success)
- **Quality**: Excellent ✓

[Repeat for all 8 images]

## Quality Summary

- {X} images generated on first attempt
- {Y} images required refinement
- Overall success rate: {X/8}%

## Files Modified

- `book-source/static/img/part-1/chapter-2/` (8 images)
- 5 lesson markdown files updated
- Audit report: `history/visual-assets/chapter-2-audit-report.md`
- Completion report: this file

---

**Chapter 2 visual assets workflow: COMPLETE** ✅
```

---

### Step 4: Final Quality Check

```bash
# Verify all images exist
ls -lh book-source/static/img/part-1/chapter-2/

# Should show 8 .png files with reasonable sizes (500KB-1MB each)
```

Start Docusaurus dev server to verify rendering:

```bash
cd book-source && npm run start
# Navigate to Chapter 2 lessons, verify all images load correctly
```

---

## BROWSER AUTOMATION RESUME (if preferred)

If continuing with Playwright automation:

1. Browser is at: https://gemini.google.com/app
2. Click Create Image button (ref in snapshot)
3. Use prompts from lesson files
4. Follow download pattern established (check `.playwright-mcp/` for latest file)
5. Copy to `book-source/static/img/part-1/chapter-2/{filename}.png`

---

## QUICK REFERENCE: Image Mapping

| #   | Filename                                  | Lesson File                 | Line |
| --- | ----------------------------------------- | --------------------------- | ---- | --- |
| 1   | evidence-compared-2024-vs-2025.png        | 01-the-inflection-point.md  | ~149 | ✅  |
| 2   | ai-capability-breakthroughs-2025.png      | 01-the-inflection-point.md  | ~86  | ✅  |
| 3   | paradigm-shift-interface-to-intent.png    | 02-user-interface-intent.md | ~136 | ⏸️  |
| 4   | five-powers-ai-agents.png                 | 02-user-interface-intent.md | ~308 | ⏸️  |
| 5   | team-comparison-vibe-vs-spec-timeline.png | 03-development-patterns.md  | ~183 | ⏸️  |
| 6   | seven-dora-capabilities-ai-success.png    | 04-dora-perspective.md      | ~140 | ⏸️  |
| 7   | three-layer-ai-development-stack.png      | 05-ai-coding-agents.md      | ~114 | ⏸️  |
| 8   | stack-evolution-2024-to-2025.png          | 05-ai-coding-agents.md      | ~247 | ⏸️  |

---

**Resume from**: Step 1, Image 3
**Browser state**: Ready at Gemini home page
**Next action**: Click Create Image button, paste prompt for Image 3
