# Quickstart: Adding Slides to Chapters

**Feature**: `035-metadata-driven-slides`
**Audience**: Content Creators
**Time**: <2 minutes per chapter

## Overview

Add PDF slides to any chapter by editing only the frontmatter (YAML metadata at the top of the README). No imports, no JSX, no build configuration needed.

---

## Basic Usage

### Step 1: Prepare Your PDF Slide Deck

1. Generate slides using NotebookLM (see `.claude/skills/notebooklm-slides/SKILL.md`)
2. Save PDF locally OR upload to cloud storage (Cloudflare R2, S3, etc.)

---

### Step 2: Add Slides to Chapter

**Option A: Local Path** (slides stored in repository)

1. Place PDF in `book-source/static/slides/`:

   ```bash
   book-source/static/slides/chapter-02-slides.pdf
   ```

2. Open chapter README (`apps/learn-app/docs/[part]/[chapter]/README.md`)

3. Add `slides` field to frontmatter:
   ```yaml
   ---
   sidebar_position: 2
   title: "Chapter 2: The AI Turning Point"
   slides: "slides/chapter-02-slides.pdf" # ← Add this line
   ---
   ```

**Option B: Cloud URL** (slides hosted on CDN)

1. Upload PDF to cloud storage
2. Get public URL (e.g., `https://r2.cloudflare.com/slides/chapter-02.pdf`)
3. Add `slides` field with full URL:
   ```yaml
   ---
   sidebar_position: 2
   title: "Chapter 2: The AI Turning Point"
   slides: "https://r2.cloudflare.com/slides/chapter-02-slides.pdf" # ← Cloud URL
   ---
   ```

---

### Step 3: Verify

1. **Start dev server**:

   ```bash
   cd book-source
   npm start
   ```

2. **Navigate to chapter** in browser

   - Slides should appear after "What You'll Learn" section
   - Should display as embedded PDF with fullscreen button

3. **Check for warnings** (console output):
   - If you see warnings, check path/URL is correct

---

## Examples

### Example 1: Simple Local Path

**File**: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/02-ai-turning-point/README.md`

```yaml
---
sidebar_position: 2
title: "Chapter 2: The AI Turning Point"
slides: "slides/chapter-02-slides.pdf"
---

# Chapter 2: The AI Turning Point

[Chapter content here...]

## What You'll Learn

[Learning objectives...]

<!-- Slides will be injected HERE automatically -->
```

**Result**:

- Import statement added automatically at top
- Slides section with PDF viewer appears after "What You'll Learn"
- No manual edits to markdown content needed

---

### Example 2: Cloud URL

**File**: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/04-nine-pillars/README.md`

```yaml
---
sidebar_position: 4
title: "Chapter 4: The Nine Pillars"
slides: "https://cdn.panaversity.org/slides/chapter-04-slides.pdf"
---

# Chapter 4: The Nine Pillars

[Chapter content here...]

## What You'll Learn

[Learning objectives...]

<!-- Slides will be injected HERE automatically -->
```

**Result**:

- Cloud URL used directly in PDFViewer src
- No local file needed in repository
- Works identically to local path approach

---

### Example 3: Chapter Without Slides

**File**: `apps/learn-app/docs/02-Foundations/05-markdown-basics/README.md`

```yaml
---
sidebar_position: 5
title: "Lesson 5: Markdown Basics"
# No slides field
---

# Lesson 5: Markdown Basics

[Lesson content here...]

## What You'll Learn

[Learning objectives...]

<!-- No slides injection (no error) -->
```

**Result**:

- No slides section appears
- No warnings or errors
- Chapter renders normally

---

## Troubleshooting

### Problem: Slides don't appear

**Possible causes**:

1. **Missing "What You'll Learn" heading**

   - Solution: Add `## What You'll Learn` section to chapter
   - Plugin uses this as injection point

2. **Incorrect path format**

   - Local path: `slides/chapter-02.pdf` (relative to static directory)
   - Cloud URL: Must start with `http://` or `https://`

3. **Typo in frontmatter field**

   - Must be exactly `slides:` (lowercase, singular)
   - Check YAML syntax (colon + space + value)

4. **File not in static directory** (local paths only)
   - Must be in `book-source/static/slides/`
   - Served at URL path `/slides/`

---

### Problem: Build warnings appear

**Warning**: `"Failed to inject slides for [file]: No target heading found"`

- **Cause**: Chapter lacks "What You'll Learn" heading
- **Solution**: Add `## What You'll Learn` section OR remove `slides` field

**Warning**: `"Failed to inject slides for [file]: Invalid slides format"`

- **Cause**: `slides` field is not a string (e.g., number, object)
- **Solution**: Use string format: `slides: "path/to.pdf"`

---

### Problem: PDF doesn't load in browser

**Local path** (`/slides/chapter-02.pdf`):

- Check file exists: `ls book-source/static/slides/chapter-02.pdf`
- Check file size (<100MB recommended for browser)
- Check browser console for 404 errors

**Cloud URL** (`https://...`):

- Verify URL is publicly accessible (test in browser)
- Check CORS headers if cross-origin
- Verify CDN authentication (if applicable)

---

## Migration Guide

### Migrating from Manual JSX to Metadata

**Before** (manual approach):

```markdown
---
sidebar_position: 2
title: "Chapter 2: The AI Turning Point"
---

import PDFViewer from '@site/src/components/PDFViewer';

# Chapter 2: The AI Turning Point

[Content...]

## 📊 Chapter Slides

<PDFViewer
  src="slides/chapter-02-slides.pdf"
  title="Chapter 2: The AI Turning Point"
  height={700}
/>
```

**After** (metadata approach):

```markdown
---
sidebar_position: 2
title: "Chapter 2: The AI Turning Point"
slides: "slides/chapter-02-slides.pdf" # ← Add this
---

# Chapter 2: The AI Turning Point

[Content...]

## What You'll Learn

[Learning objectives...]

<!-- Slides injected automatically - remove manual JSX -->
```

**Steps**:

1. Add `slides` field to frontmatter
2. Remove manual `import PDFViewer` statement
3. Remove manual `<PDFViewer>` JSX
4. Ensure "What You'll Learn" section exists
5. Test in dev server

---

## Best Practices

### 1. Consistent Naming Convention

**Recommended format**: `chapter-{number}-slides.pdf`

```
book-source/static/slides/
├── chapter-01-slides.pdf
├── chapter-02-slides.pdf
├── chapter-03-slides.pdf
└── ...
```

### 2. PDF Optimization

- **File size**: <20MB (faster loading)
- **Resolution**: 1920×1080 or 1280×720 (readable on most screens)
- **Compression**: Use PDF tools to reduce size without quality loss

### 3. Cloud Storage

When using cloud URLs:

- **Use CDN**: Ensure fast global delivery (Cloudflare R2, AWS CloudFront)
- **Set cache headers**: Long-lived cache (slides rarely change)
- **Public access**: No authentication required for reading

### 4. Version Control

- **Local files**: Commit PDFs to repository (if <10MB each)
- **Large files**: Use Git LFS OR cloud storage + URL in frontmatter
- **Cloud-first**: If PDFs are large (>10MB), prefer cloud URLs over local files

---

## FAQ

**Q: Can I have multiple slide decks per chapter?**
A: Not currently. The `slides` field accepts a single string. If multiple decks needed, combine PDFs before upload.

**Q: Can I customize the height of the PDF viewer?**
A: Currently fixed at 700px. Future enhancement: support object format with options.

**Q: What happens if I use both metadata AND manual JSX?**
A: Both will render (backward compatibility). Manual JSX appears where you place it, metadata-driven slides appear after "What You'll Learn".

**Q: Can I disable slides injection for a single chapter?**
A: Yes - simply omit the `slides` field from frontmatter.

**Q: Do I need to rebuild when changing slides PDF?**
A: No - if using cloud URL, update PDF on CDN (browser cache may delay). For local files, Docusaurus will hot-reload when file changes.

**Q: Can I use relative paths like `./slides/chapter.pdf`?**
A: Use `slides/chapter.pdf` (no leading `./`). Plugin normalizes to `/slides/chapter.pdf`.

---

## Summary

**Add slides in 3 steps**:

1. Place PDF in `static/slides/` OR upload to cloud
2. Add `slides: "path"` to frontmatter
3. Verify in dev server

**Zero manual edits** to markdown content. Plugin handles imports and JSX injection automatically.

For advanced usage or troubleshooting, see:

- Technical architecture: [`plan.md`](./plan.md)
- Data model: [`data-model.md`](./data-model.md)
- Implementation: [`tasks.md`](./tasks.md) (after Phase 2)
