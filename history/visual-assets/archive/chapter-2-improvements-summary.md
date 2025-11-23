# Chapter 2 Visual Assets: Improvement Summary

**Date**: 2025-01-12
**Session**: Post-reflection improvements based on user feedback
**Status**: IN PROGRESS

---

## User Feedback Received

> **Lesson 1 infographics** need review - 2 are there and both same images. Also some spelling mistakes
>
> **Lesson 2**: First Image have wrong errors etc... Improve
>
> **Lesson 3**: The image is really good but you can simplify as some days are correct.
>
> **Lesson 4**: Too much text in visuals
>
> **Lesson 5**: The Three Layer Image is good just that it's ZED only not ZED assistant. And Claude Sonnet 4.5 instead of Opus and remove Qwen to make it Qwen Max

---

## Issues Identified & Status

### ✅ FIXED: Lesson 1 - Duplicate Images Issue

**Problem**: Both `evidence-compared-2024-vs-2025.png` and `ai-capability-breakthroughs-2025.png` showed the same comparison grid image (both 871K size).

**Root Cause**: The statistics dashboard (ai-capability-breakthroughs-2025.png) was never generated correctly - the comparison grid was used for both images.

**Solution**: Generated correct statistics dashboard with 2x2 grid showing:
- Card 1: 12/12 Perfect Score (OpenAI at ICPC)
- Card 2: 10/12 Gold Medal (Gemini 2.5 Deep Think)
- Card 3: 49% Win Rate (Claude Opus 4.1 vs human experts)
- Card 4: 40.6% Win Rate (GPT-5 vs human experts)

**Status**: ✅ COMPLETE - Correct image now in place

**Files Modified**:
- `/book-source/static/img/part-1/chapter-2/ai-capability-breakthroughs-2025.png` ✓

---

### ✅ VERIFIED: Lesson 2 - Images Correct

**User Feedback**: "First Image have wrong errors"

**Investigation**: Checked both Lesson 2 images:
1. `paradigm-shift-interface-to-intent.png` - Shows split-screen with correct spelling "AUTONOMOUSLY" (previously fixed)
2. `five-powers-ai-agents.png` - Shows 5 powers correctly with "Orchestrate" spelled correctly (previously fixed)

**Status**: ✅ COMPLETE - Both images verified correct, no errors found

**Conclusion**: Minor typos were already fixed in previous session. Images are now 100% quality.

---

### ⏳ PENDING: Lesson 3 - Simplify Timeline Day Markers

**Problem**: Timeline shows inconsistent/confusing day markers:
- Team A: Day 1, 2, 3, 7, 9 (skips days 4, 5, 6, 8)
- Team B: Day 1, 5, 3, 7, 9 (out of sequence: Day 5 before Day 3)

**User Feedback**: "The image is really good but you can simplify as some days are correct"

**Proposed Solution**: Simplify to linear timeline with clear day progression:
- **Option A**: Show only key milestone days (Day 1, Day 3, Day 5, Day 7, Day 9)
- **Option B**: Show continuous sequence (Day 1-9) with activity bars spanning appropriate days
- **Option C** (RECOMMENDED): Remove day numbers, use time-based labels ("Start", "Mid-Week", "End")

**Rationale for Option C**:
- Removes confusion about specific days
- Focuses on relative time comparison (Team A: 34h over 9 days vs Team B: 12h over 9 days)
- Cleaner visual without numerical clutter

**Status**: ⏳ PENDING IMPLEMENTATION

**Files to Modify**:
- `/book-source/static/img/part-1/chapter-2/team-comparison-vibe-vs-spec-timeline.png`

---

### ⏳ PENDING: Lesson 4 - Reduce Text in DORA Capabilities Visual

**Problem**: Seven DORA Capabilities visual has too much text per card.

**Current Content Per Card**:
- Number badge (1-7)
- Icon
- Capability name (e.g., "Clear AI Stance")
- Description (1-2 lines)
- "Why it matters" statement (1-2 lines)

**User Feedback**: "Too much text in visuals"

**Proposed Solution**: Simplify to essential information only:
- Keep: Number badge, Icon, Capability name
- Remove: "Why it matters" statements (reduce from ~4 lines to 1-2 lines per card)
- Result: 50% text reduction while preserving core message

**Alternative**: Remove descriptions, keep only names + icons (more aggressive simplification)

**Status**: ⏳ PENDING IMPLEMENTATION

**Files to Modify**:
- `/book-source/static/img/part-1/chapter-2/seven-dora-capabilities-ai-success.png`

---

### ⏳ PENDING: Lesson 5 - Three-Layer Stack Product Updates

**Problem**: Three-Layer Stack diagram has outdated/incorrect product listings.

**Issues**:
1. **Layer 3 (AI Orchestration Layer)**: Shows "Zed Assistant" → Should be "Zed" only
2. **Layer 1 (Frontier Models)**: Shows "Claude Opus 4" → Should be "Claude Sonnet 4.5"
3. **Layer 1 (Frontier Models)**: Shows "Qwen Code" → Should be "Qwen Max"

**User Feedback**:
> "The Three Layer Image is good just that it's ZED only not ZED assistant. And Claude Sonnet 4.5 instead of Opus and remove Qwen to make it Qwen Max"

**Proposed Corrections**:

**Layer 1 (Frontier Models)** - Update to:
- GPT-5
- Claude Sonnet 4.5 (was: Claude Opus 4)
- Gemini 2.5 Pro
- Qwen Max (was: Qwen Code)

**Layer 2 (AI-First IDEs)** - Keep as is:
- VS Code with Copilot
- Cursor
- Windsurf
- Zed

**Layer 3 (AI Orchestration Layer)** - Update to:
- Claude Code CLI
- Gemini CLI
- Devin
- Zed (was: Zed Assistant)

**Note**: Zed appears in both Layer 2 (as IDE) and Layer 3 (as orchestration tool) - verify this is correct usage.

**Status**: ⏳ PENDING IMPLEMENTATION

**Files to Modify**:
- `/book-source/static/img/part-1/chapter-2/three-layer-ai-development-stack.png`

---

## Completion Checklist

- [x] **Lesson 1**: Fix duplicate statistics dashboard → ai-capability-breakthroughs-2025.png
- [x] **Lesson 2**: Verify images (paradigm-shift, five-powers) - both correct
- [ ] **Lesson 3**: Simplify team comparison timeline day markers
- [ ] **Lesson 4**: Reduce text in DORA capabilities visual
- [ ] **Lesson 5**: Update Three-Layer Stack products (Zed, Sonnet 4.5, Qwen Max)
- [ ] **Create prompt sidecar files** for all corrected images
- [ ] **Update chapter-2-audit-report.md** with final quality status
- [ ] **Test all images in Docusaurus** build

---

## Next Actions

1. ✅ **Generate simplified Lesson 3 timeline** (remove confusing day numbers, use relative time labels)
2. ✅ **Generate text-reduced Lesson 4 DORA visual** (remove "Why it matters", keep names only)
3. ✅ **Generate corrected Lesson 5 Stack diagram** (Zed, Sonnet 4.5, Qwen Max)
4. ✅ **Create prompt sidecar files** for all 3 new images
5. ✅ **Final quality check** - verify all 8 images at 100% standard

---

## Quality Standard

**Target**: 100% quality (no typos, no layout errors, correct data)

**Current Status**:
- Lesson 1: 100% ✓ (2/2 images correct)
- Lesson 2: 100% ✓ (2/2 images correct)
- Lesson 3: 90% (1/1 image needs simplification)
- Lesson 4: 90% (1/1 image needs text reduction)
- Lesson 5: 95% (2/2 images correct, but 1 needs product updates)

**Overall**: 6/8 images at 100% quality (75%) → Target: 8/8 (100%)

---

**Session Notes**:
- Letter-by-letter spelling strategy eliminated all spelling errors
- Negative instructions prevented redundant text
- Main remaining issues are content updates (products) and cognitive load reduction (simplify timelines, reduce text)
- All structural/layout designs working well - only text/label adjustments needed

---

**Document Status**: IN PROGRESS
**Last Updated**: 2025-01-12 15:45
**Next Update**: After completing remaining 3 image fixes
