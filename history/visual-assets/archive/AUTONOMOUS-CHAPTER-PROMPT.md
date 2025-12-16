# Autonomous Chapter Visual Assets Workflow Prompt

## Single-Prompt Autonomous Flow

Copy and paste this prompt in a fresh Claude Code session to execute the complete visual assets workflow for any chapter:

---

**AUTONOMOUS VISUAL ASSETS WORKFLOW - CHAPTER 2**

Execute the complete visual assets workflow for Chapter 2 autonomously from planning through final image generation. I will only review at the end before pushing to git.

**Your autonomous tasks:**

1. **Read all Chapter 2 lesson files** (`apps/learn-app/docs/01-Introducing-AI-Driven-Development/02-*/`)

2. **Plan visual assets** (use `visual-asset-workflow` skill):

   - Audit all 6 lessons for visual opportunities
   - Apply pedagogical value assessment (PASS/CONDITIONAL/REJECT)
   - Check redundancy across lessons (skip duplicates)
   - Apply Chapter 1 learnings:
     - Bar charts: 3 bars maximum
     - Compound words: Use hyphenated versions ("Auto-Complete")
     - Indicators: Arrow-only (no text labels)
     - Labels: Minimal essential text only
   - Generate image prompts and embed in lesson markdown
   - Create audit report in `history/visual-assets/chapter-2-audit-report.md`

3. **Generate all images** (use `image-generator` skill):

   - Open Gemini.google.com (I'll log in once)
   - For each visual asset:
     - Start fresh session (click "New chat")
     - Generate image
     - Verify immediately with screenshot (check spelling, layout, colors)
     - If issues: Apply decision tree (hyphenate → fresh session → simplify)
     - Download when 99% quality achieved
     - Update lesson markdown (replace HTML comment with image)
   - Create completion report in `history/visual-assets/chapter-2-visual-assets-report.md`

4. **Final quality check**:

   - Read all updated lesson files
   - Verify all images render correctly
   - Check for spelling errors in images
   - Confirm no redundant visuals
   - Validate pedagogical value

5. **Present for review**:
   - Show me summary of:
     - Total visuals created (by lesson)
     - Any issues encountered and how resolved
     - Quality assessment (first-attempt success rate)
     - Files modified
   - Ask: "Ready to commit and push? [Y/n]"

**Quality standards:**

- 99% target (only accept perfect spelling, clean layout)
- Apply fresh session technique if same error persists 2+ times
- Simplify complexity if 3+ iterations fail with same approach
- Document all learnings in completion report

**What I'll do:**

- Log into Gemini once at start
- Final review before git push
- That's it - you handle everything else autonomously

**Start now. Begin with: "Starting autonomous Chapter 2 visual assets workflow..."**

---

## Usage Instructions

1. Open fresh Claude Code session
2. Copy entire prompt above (starting from "AUTONOMOUS VISUAL ASSETS WORKFLOW")
3. Paste and send
4. Log into Gemini when prompted
5. Wait for completion and review summary
6. Approve git push when ready

## Customization for Other Chapters

To use for different chapters, change:

- "CHAPTER 2" → "CHAPTER X"
- Path: `02-*` → `0X-*`
- Report filenames: `chapter-2-*` → `chapter-X-*`

## Expected Timeline

- Planning: 5-10 minutes
- Image generation: 2-4 minutes per image (depends on complexity and iterations)
- Total: 20-40 minutes for typical chapter (4-6 visuals)

## What Makes This Autonomous

The prompt gives Claude Code:

- ✅ Clear end-to-end workflow
- ✅ Quality gates with decision authority
- ✅ Chapter 1 learnings encoded
- ✅ Single human checkpoint (final review)
- ✅ Permission to iterate and refine
- ✅ Documentation requirements

You only intervene at:

1. Gemini login (one-time)
2. Final review and git approval

Everything else runs autonomously following proven Chapter 1 strategies.
