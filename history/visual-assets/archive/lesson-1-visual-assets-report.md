# Visual Assets - Chapter 1, Lesson 1

**Status**: ✅ Prompts embedded in markdown
**Date**: 2025-01-12
**Total Assets**: 3

---

## Asset Summary

### VISUAL ASSET 1: YC Winter 2025 AI-Generated Code Infographic
- **Location**: Line 62 (after opening paragraphs, before "This pattern is appearing...")
- **Type**: Single metric infographic with grid visualization
- **Dimensions**: 1024x1024 (1:1)
- **Purpose**: Establish credibility with headline YC W25 statistic
- **Priority**: HIGH
- **Filename**: `yc-w25-ai-generated-code-stats.png`
- **Key Visual**: 10x10 grid of startup icons, 25 highlighted in orange

---

### VISUAL ASSET 2: AI Adoption Statistics Dashboard (2025)
- **Location**: Line 123 (after 4-bullet statistics list, before "But numbers alone...")
- **Type**: 2x2 metric card dashboard
- **Dimensions**: 1792x1024 (16:9)
- **Purpose**: Make dense statistics scannable and digestible
- **Priority**: HIGH
- **Filename**: `ai-adoption-statistics-dashboard-2025.png`
- **Key Visual**: 4 metric cards showing 84%, $500M, 75%, 7.5%

---

### VISUAL ASSET 3: Technology Adoption Speed Comparison Timeline
- **Location**: Line 117 (after paragraph about "months not years", before statistics)
- **Type**: Horizontal bar chart / timeline
- **Dimensions**: 1792x1024 (16:9)
- **Purpose**: Emphasize unprecedented speed of AI adoption vs. history
- **Priority**: MEDIUM
- **Filename**: `technology-adoption-speed-comparison.png`
- **Key Visual**: 4 long gray bars (10-15 years) vs. 1 short orange bar (months)

---

## Generation Instructions

### Step 1: Generate Images
Use any AI image generation tool:
- **DALL-E 3** (OpenAI): Copy full prompt from HTML comment
- **Imagen 3** (Google): Copy full prompt from HTML comment
- **Midjourney**: Adapt prompt (add `--ar 16:9` or `--ar 1:1`, `--style raw`)

### Step 2: Save Files
Save generated images to:
```
/book-source/static/img/part-1/chapter-1/
```

With exact filenames:
- `yc-w25-ai-generated-code-stats.png`
- `ai-adoption-statistics-dashboard-2025.png`
- `technology-adoption-speed-comparison.png`

### Step 3: Replace HTML Comments with Images
In `01-moment_that_changed_everything.md`, replace each `<!-- VISUAL ASSET N: ... -->` block with:

```markdown
![Alt text here](/img/part-1/chapter-1/filename.png)
```

Using the alt text provided in each prompt.

### Step 4: Verify Rendering
```bash
cd book-source
npm run start
```

Navigate to Chapter 1, Lesson 1 and verify:
- Images load correctly
- Alt text displays on hover
- Images are appropriately sized
- Visual quality is high-resolution

---

## Design System Reference

All 3 assets use consistent design language:

**Color Palette**:
- Primary accent: Orange (#FF6B35)
- Secondary accent: Blue (#0066FF)
- Background: White (#FFFFFF) or Light Gray (#F8F9FA)
- Text primary: Dark Gray (#1A1A1A)
- Text secondary: Medium Gray (#666666)
- Text tertiary: Light Gray (#999999)

**Typography**:
- Font family: Roboto
- Headings: Bold or Medium
- Body: Regular
- Sizes: 72pt (metrics), 32pt (titles), 18-24pt (labels), 10-14pt (captions)

**Style**:
- Modern tech publication aesthetic
- Flat design with subtle shadows
- Clean, minimal, data-focused
- Generous white space

---

## Quality Checklist

Before finalizing, verify each image:

- [ ] All numbers match source data exactly
- [ ] Text is readable at book size (minimum 10pt)
- [ ] Colors match design system (hex codes verified)
- [ ] Layout is balanced and professional
- [ ] No typos or text errors
- [ ] High resolution (300 DPI for print, 144 DPI minimum for digital)
- [ ] File size reasonable (<500KB per image)
- [ ] Alt text is descriptive and accurate

---

## Next Steps

1. **Generate Asset 2 first** (statistics dashboard) - highest priority, establishes visual style
2. **Generate Asset 1 second** (YC infographic) - simpler, uses similar design language
3. **Generate Asset 3 last** (timeline) - medium priority, different format

**Estimated time**: 30-45 minutes for all 3 assets (including iteration)

**Tools recommended**:
- DALL-E 3 via ChatGPT Plus (good at text rendering)
- Imagen 3 via Google AI Studio (good at layouts)
- Midjourney (good at style consistency, may need text cleanup)

---

## Notes

- All prompts include exact specifications (dimensions, colors, typography, layout)
- Prompts are ready to copy-paste into image generation tools
- Each prompt includes filename suggestion and alt text
- Design system is consistent across all 3 assets for visual coherence
