# NotebookLM Slides - Batch Generation Workflow

## Current Status

### ✅ Generating Now (Ready for Download):

- **Part 3 (Chapters 10-12)**: All 3 chapters have sources uploaded and slides are generating
  - Chapter 10: Markdown - The Language of AI Communication (6 sources)
  - Chapter 11: Prompt Engineering for AI-Driven Development (9 sources)
  - Chapter 12: Context Engineering for AI-Driven Development (10 sources)

### 📋 To Be Created:

- **Part 1**: Chapters 1-4 (4 chapters)
- **Part 2**: Chapters 5-9 (5 chapters)
- **Part 4**: Chapters 13-30 (18 chapters)
- **Part 5**: Chapters 31-34 (4 chapters)

**Total to Create**: 31 chapters
**Total All Chapters**: 34 chapters

---

## Efficient Batch Workflow

### Step 0: Complete Part 3 (PRIORITY - Already Generating!)

**Part 3 is already set up and generating slides. Complete these first:**

1. Wait for generation to complete (~2-5 min per chapter)
2. Download PDFs from each notebook
3. Rename and organize (see Step 4 below)
4. Integrate into READMEs (see Step 5 below)

**NotebookLM URLs for Part 3:**

- Chapter 10: Check your browser tab "Chapter 10: Markdown - The Language of AI Communication 📝"
- Chapter 11: Check your browser tab "Chapter 11: Prompt Engineering for AI-Driven Development 💬"
- Chapter 12: Check your browser tab "Chapter 12: Context Engineering for AI-Driven Development 🧠"

---

### Step 1: Create Notebooks in Batches of 3-5

Open NotebookLM (https://notebooklm.google.com) and create notebooks in batches:

**Batch 1 - Part 1 (Chapters 1-4):**

1. Chapter 1: AI Development Revolution
2. Chapter 2: The AI Turning Point
3. Chapter 3: Billion-Dollar AI Strategies
4. Chapter 4: The Nine Pillars of AIDD

**Batch 2 - Part 2 (Chapters 5-9):**

1. Chapter 5: Claude Code Features
2. Chapter 6: Gemini CLI Installation
3. Chapter 7: Bash Essentials
4. Chapter 8: AI-Native IDEs
5. Chapter 9: Git and GitHub

**Batch 3-8 - Part 4 (Chapters 13-30):**
Split into 3-chapter batches for efficiency

**Batch 9 - Part 5 (Chapters 31-34):**
All 4 chapters together

---

## Step 2: Upload Sources for Each Chapter

For each chapter, upload ALL lesson markdown files + README.md:

### File Locations by Chapter:

**Part 1:**

- Ch 1: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/*.md`
- Ch 2: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/02-ai-turning-point/*.md`
- Ch 3: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/03-billion-dollar-ai/*.md`
- Ch 4: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/04-nine-pillars/*.md`

**Part 2:**

- Ch 5: `apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/*.md`
- Ch 6: `apps/learn-app/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/*.md`
- Ch 7: `apps/learn-app/docs/02-AI-Tool-Landscape/07-bash-essentials/*.md`
- Ch 8: `apps/learn-app/docs/02-AI-Tool-Landscape/08-ai-native-ides/*.md`
- Ch 9: `apps/learn-app/docs/02-AI-Tool-Landscape/09-git-and-github/*.md`

**Part 3 (✅ Already Uploaded):**

- Ch 10: `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/10-markdown-language-of-ai/*.md`
- Ch 11: `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/11-prompt-engineering-for-aidd/*.md`
- Ch 12: `apps/learn-app/docs/03-Markdown-Prompt-Context-Engineering/12-context-engineering-for-ai-driven-development/*.md`

**Part 4:**

- Ch 13: `apps/learn-app/docs/04-Python-Fundamentals/13-python-uv-package-manager/*.md`
- Ch 14-30: (similar pattern)

**Part 5:**

- Ch 31: `apps/learn-app/docs/05-Spec-Driven-Development/31-specification-driven-development-fundamentals/*.md`
- Ch 32-34: (similar pattern)

---

## Step 3: Generate Slides (Use Default Settings)

For each notebook:

1. Click **"Slide Deck"** in Studio panel
2. **Skip custom prompts** - use default generation (NotebookLM will auto-generate based on sources)
3. Click **"Generate"**
4. Wait 2-5 minutes per chapter

**Pro Tip**: Open multiple tabs (3-5 notebooks) and start generation in parallel for efficiency!

---

## Step 4: Download and Organize

Once slides are generated:

1. **Download PDF** from each notebook
2. **Rename** following this pattern:

   - `chapter-01-slides.pdf`
   - `chapter-02-slides.pdf`
   - etc.

3. **Move to**: `book-source/static/slides/`

---

## Step 5: Integrate into Documentation

Add PDFViewer component to each chapter's README.md:

```markdown
import PDFViewer from '@site/src/components/PDFViewer';

## 📊 Chapter Slides

<PDFViewer
  src="slides/chapter-[NUMBER]-slides.pdf"
  title="Chapter [NUMBER]: [Chapter Title]"
  height={700}
/>
```

**Location**: Add after "What You'll Learn" section in each chapter's README

---

## Quick Reference Commands

### Find all lesson files for a chapter:

```bash
# Example for Chapter 1
find apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution -name "*.md" -type f ! -name "*quiz*" | sort
```

### Count lessons per chapter:

```bash
# Example for Chapter 1
find apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution -name "*.md" -type f ! -name "*quiz*" | wc -l
```

### Verify slides directory:

```bash
ls -la book-source/static/slides/
```

---

## Time Estimates

- **Notebook creation**: ~1 min per chapter
- **Source upload**: ~2-3 min per chapter
- **Slide generation**: ~3-5 min per chapter
- **Download & organize**: ~1 min per chapter
- **README integration**: ~1 min per chapter

**Total per chapter**: ~8-11 minutes
**Total for Part 3 (Chapters 10-12)**: ✅ Done! Just need to download & integrate (~15 min)
**Total for remaining 31 chapters**: ~4-6 hours (with parallel processing: ~3-4 hours)
**Grand Total (34 chapters)**: ~4.5-6.5 hours

---

## Automation Script (Optional)

You could automate file collection with a script like this:

```bash
#!/bin/bash

# Example: Collect all lesson files for Chapter 1
CHAPTER_DIR="apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution"
OUTPUT_DIR="temp_chapter_sources"

mkdir -p "$OUTPUT_DIR"
find "$CHAPTER_DIR" -name "*.md" -type f ! -name "*quiz*" -exec cp {} "$OUTPUT_DIR/" \;

echo "Files ready for upload in: $OUTPUT_DIR"
```

---

## Success Criteria

✅ Each chapter should have:

- Notebook created with descriptive title
- All lesson files + README uploaded
- Slide deck generated (12-20 slides typical)
- PDF downloaded and renamed correctly
- PDFViewer component added to README
- Slides accessible on Docusaurus site

---

## Notes

- **No custom prompts needed**: Default generation works well for most chapters
- **Parallel processing**: Open 3-5 tabs to maximize efficiency
- **Source limit**: 300 sources per notebook (we're well under this for all chapters)
- **File format**: NotebookLM supports Markdown directly
- **Browser**: Works best in Chrome/Edge (Google's recommendation)
