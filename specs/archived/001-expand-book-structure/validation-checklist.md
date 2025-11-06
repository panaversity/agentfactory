# Cross-Reference Validation Checklist

**Created**: 2025-10-29  
**Purpose**: Ensure all internal references are updated after book structure expansion  
**Use**: Run through this checklist after Phase 8 (Polish) to verify consistency

---

## Files That Reference Chapter Numbers

### Core Documentation
- [ ] `.specify/memory/constitution.md` — References to parts, chapters, scaffolding ranges
- [ ] `specs/book/chapter-index.md` — All chapter titles and numbers
- [ ] `specs/book/directory-structure.md` — Example paths and folder names
- [ ] `README.md` — Structure overview section
- [ ] `CLAUDE.md` — Should defer to constitution (verify no hardcoded numbers)

### Part Specifications
- [ ] `specs/part-1/part-1-spec.md` — Chapter breakdown and references
- [ ] `specs/part-2/` (if exists) — Check all chapter references
- [ ] `specs/part-3/` (if exists) — Check all chapter references
- [ ] `specs/part-4/` (if exists) — Check all chapter references
- [ ] `specs/part-5/` (if exists) — Check all chapter references
- [ ] `specs/part-6/part-6-spec.md` (after creation) — Check references
- [ ] `specs/part-7/part-7-spec.md` (after creation) — Check references
- [ ] `specs/part-8/part-8-spec.md` (after creation) — Check references
- [ ] `specs/part-9/part-9-spec.md` (after creation) — Check references
- [ ] `specs/part-10/part-10-spec.md` (after creation) — Check references
- [ ] `specs/part-11/part-11-spec.md` (after creation) — Check references
- [ ] `specs/part-12/part-12-spec.md` (after creation) — Check references
- [ ] `specs/part-13/part-13-spec.md` (after creation) — Check references

### Docusaurus Content
- [ ] `book-source/docs/01-Introducing-AI-Driven-Development/intro.md` — Part intro references
- [ ] `book-source/docs/02-AI-Tool-Landscape/intro.md` — Part intro references
- [ ] `book-source/docs/03-Prompt-and-Context-Engineering/intro.md` — Part intro references
- [ ] `book-source/docs/04-Python-The-Language-of-AI-Agents/intro.md` — Part intro references
- [ ] `book-source/docs/05-Spec-Kit-Plus-Methodology/intro.md` — Part intro references
- [ ] `book-source/docs/06-Agentic-AI-with-OpenAI-Agents-SDK/intro.md` — Part intro references
- [ ] `book-source/docs/07-MCP-with-FastMCP/intro.md` — Part intro references
- [ ] All chapter README.md files — Cross-references to other chapters

---

## Internal Link Validation Procedure

### Step 1: Search for Old Chapter Numbers

Run these searches in your IDE or command line:

```bash
# Search for references to old chapter numbers that may need updating
grep -r "Chapter [1-9]" book-source/docs/ specs/part-*/
grep -r "Ch [1-9]" book-source/docs/ specs/part-*/
grep -r "chapter [1-9]" book-source/docs/ specs/part-*/

# Search for old part references
grep -r "Part [1-7]" book-source/docs/ specs/
grep -r "7 parts" .
grep -r "32 chapters" .
```

### Step 2: Verify Markdown Links

```bash
# Find all markdown links to chapters
grep -r "\[.*\](.*chapter.*)" book-source/docs/

# Find all markdown links to parts
grep -r "\[.*\](.*part.*)" book-source/docs/
```

### Step 3: Check for Hardcoded Paths

```bash
# Find absolute paths that may be outdated
grep -r "book-source/docs/[0-9]" .

# Find references to specific chapter folders
grep -r "/[0-9][0-9]-.*/" book-source/docs/
```

---

## Docusaurus Build Validation

### Step 1: Clean Build

```bash
cd book-source
rm -rf build/ .docusaurus/
npm run build
```

**Expected Result**: Build succeeds with zero errors

### Step 2: Sidebar Validation

1. Open `http://localhost:3000` (after `npm start`)
2. Check sidebar shows all 13 parts
3. Verify all 46 chapters are listed
4. Confirm correct ordering (1-46)
5. Check all chapter titles match chapter-index.md

### Step 3: Navigation Validation

1. Click through each part in sidebar
2. Verify each chapter page loads
3. Check no broken links on chapter pages
4. Verify "Previous" and "Next" navigation works

### Step 4: Search Validation

1. Search for chapter titles
2. Verify results link to correct chapters
3. Check no orphaned pages in search results

---

## Manual Verification Checklist

### Constitution Consistency

- [ ] Constitution version updated to 2.2.0
- [ ] All "7 parts, 32 chapters" replaced with "13 parts, 46 chapters"
- [ ] Scaffolding ranges updated: (1-9, 10-30, 31-46)
- [ ] No contradictions between constitution and chapter-index

### Chapter Index Accuracy

- [ ] Header states "46 chapters across 13 parts"
- [ ] All 46 chapters listed with correct global numbers
- [ ] Part assignments match constitution
- [ ] Chapter titles match Docusaurus folder names

### Directory Structure Compliance

- [ ] All part folders follow naming: `NN-Title-Case-With-Hyphens/`
- [ ] All chapter folders follow naming: `NN-lowercase-with-hyphens/`
- [ ] Each part has `intro.md`
- [ ] Each chapter has `README.md`
- [ ] No orphaned folders or files

### Content Preservation

- [ ] Git history shows all old content preserved
- [ ] Backup tag `backup-part1-5chapters` exists
- [ ] chapter-mapping.md accounts for all 32 old chapters
- [ ] No content lost during consolidation

---

## Automated Validation Scripts (Future Enhancement)

Create these scripts to automate validation:

### `validate-chapter-refs.sh`
```bash
#!/bin/bash
# Check all markdown files for outdated chapter references
# Flag any references to chapters > 32 that aren't in the new structure
```

### `validate-links.sh`
```bash
#!/bin/bash
# Use markdown-link-check or similar to verify all internal links
```

### `validate-structure.sh`
```bash
#!/bin/bash
# Verify directory structure matches directory-structure.md spec
```

---

## Validation Sign-Off

When all items are checked:

- [ ] All chapter references updated
- [ ] All internal links verified
- [ ] Docusaurus build succeeds
- [ ] All 13 parts navigable
- [ ] All 46 chapters accessible
- [ ] No broken links found
- [ ] No orphaned content
- [ ] Git history clean and tagged

**Validated By**: ________________  
**Date**: ________________  
**Ready for**: Merge to main / Production deployment

---

**Status**: Template checklist, ready for use after Phase 8 completion

