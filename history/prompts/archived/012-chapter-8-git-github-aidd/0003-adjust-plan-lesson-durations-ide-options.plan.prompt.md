---
id: 0003
title: Adjust Plan Lesson Durations IDE Options
stage: plan
date: 2025-11-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 012-chapter-8-git-github-aidd
branch: 012-chapter-8-git-github-aidd
user: HP
command: /sp.plan (iteration)
labels: ["chapter-8", "git", "github", "plan-refinement", "lesson-duration", "ide-options", "cognitive-load", "beginner-friendly"]
links:
  spec: specs/012-chapter-8-git-github-aidd/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/012-chapter-8-git-github-aidd/plan.md (updated)
tests:
  - Lesson duration adjustment validation (PASS)
  - IDE options cognitive load check (PASS)
  - Capstone simplification for non-coders (PASS)
  - Time estimates consistency check (PASS)
---

## Prompt

1. I think one lesson should be between 15 to 20 mins is enough what you think but dont want to lose context as well and dont want to be so long that user fed up and leave so 15 mins are reasonable. 2. no need to split them but gave heading of these topics in one lesson 3. In IDE go for VS code, cursor and Zed no need to talk about anything else. 4. just mention about it in one paragraph in suitable lesson would be better. 5. Calculator project is good option but donot assume that user know coding at this point. 6. Instead of screenshots use links of installation and if you can add screenshots you can because I dont have any screenshots. If not use other alternative method to describe to students. Make adjustment to plan based on above answers

## Response snapshot

Updated Chapter 8 implementation plan with comprehensive refinements based on user's 6 requirements:

**1. Lesson Duration Adjustment (15-20 minutes)**:
- **Lesson 1**: Already 15 min (no change needed)
- **Lesson 2**: 25 min → 20 min (focused on essential installation)
- **Lesson 3**: 25 min → 20 min (streamlined daily workflow)
- **Lesson 4**: 30 min → 20 min (focused on most-used undo methods)
- **Lesson 5**: 25 min → 20 min (already updated earlier)
- **Lesson 6**: 25 min → 20 min (already updated earlier)
- **Lesson 7**: 30 min → 20 min (focused on essential PR workflow)
- **Lesson 8**: 25 min → 20 min (streamlined IDE setup)
- **Lesson 9**: 30 min → 20 min (simplified capstone for non-coders)

**2. Lesson Structure**:
- Maintained single lessons with clear headings (no splitting)
- Added notes to each lesson explaining focus areas

**3. IDE Options Reduced to 3**:
- **Removed**: PyCharm Community (to respect Tier 1 cognitive load)
- **Kept**: VS Code (recommended), Cursor, Zed
- Updated IDE comparison table from 4 to 3 options
- Updated QA checklist to verify "exactly 3 IDE options"

**4. Natural Language Prompts**:
- Added explanatory paragraph in Lesson 6 (GitHub Integration)
- Explains AIDD principle: "understand concepts, let AI handle syntax"
- References usage throughout Part II (Lessons 5-9)

**5. Capstone Simplified for Non-Coders**:
- Emphasized "AI generates ALL code - students focus on Git workflow"
- Updated scenario: "AI writes ALL code; you manage Git for safety"
- Modified steps to show **You** (student) and **AI** (code generator) roles
- Clarified: Students don't write code, just manage Git operations
- Updated focus statements: "Understanding Git workflow, NOT writing code"

**6. Installation Guidance Without Screenshots**:
- Changed from "Screenshots" to "Installation links with text descriptions"
- Updated accessibility requirements
- Reduced review effort from "Screenshots captured" to "Installation link verification"
- Specified text-based descriptions for all installation steps

**Time Estimates Updated**:
- **Part I** (Lessons 1-4): 75 minutes (15+20+20+20)
- **Part II** (Lessons 5-9): 100 minutes (20+20+20+20+20)
- **Total**: 175 minutes = 3 hours (down from 3.5-4 hours)

**Effort Estimates Revised**:
- **Writing**: 22-31 hours (down from 30-39 hours)
- **Review**: 7-11 hours (down from 9-13 hours)
- **Grand Total**: 29-42 hours (down from 39-52 hours)
- **Reductions due to**: Shorter lessons, 3 IDE options, no screenshots, simplified capstone

**Key Updates Made**:
1. Chapter-level time estimate: "3 hours (175 minutes total: Part I 75 min + Part II 100 min)"
2. Lesson Architecture section: Updated to show 75 min + 100 min breakdown
3. Part I Summary: 75 minutes (15+20+20+20)
4. Part II Summary: 100 minutes, includes Lesson 9 capstone
5. IDE Comparison Table: 3 options with "Tier 1 cognitive load" note
6. Capstone Steps: Labeled **You** vs **AI** roles, emphasized AI does all coding
7. Effort Estimates: Detailed notes on reductions
8. QA Checklist: "exactly 3 IDE options", "installation links verified", "text-based descriptions"
9. Inclusivity Requirements: "Three IDE options... while respecting Tier 1 cognitive load"
10. Pedagogical Approach: Updated Part II description to include Lessons 5-9

## Outcome

- ✅ Impact: Plan now fully aligned with beginner-friendly approach - manageable lesson durations, reduced cognitive load, simplified for absolute non-coders, accessible without screenshots
- 🧪 Tests: Lesson duration validation PASS (all 15-20 min), IDE cognitive load check PASS (3 options only), Capstone non-coder check PASS (AI generates all code), Time consistency PASS (175 min total)
- 📁 Files: Updated plan.md with 10+ sections modified, all time estimates aligned, effort estimates reduced appropriately
- 🔁 Next prompts: Plan ready for `/sp.tasks` to generate actionable implementation tasks. Then proceed with content-implementer implementation phase.
- 🧠 Reflection: Critical refinements for true beginners. Original plan assumed some technical comfort. Updated version respects cognitive limits (3 IDE options, not 4), realistic time expectations (15-20 min lessons maintain engagement), and acknowledges audience may have zero coding experience (AI generates all capstone code). These adjustments significantly improve accessibility and completion likelihood.

## Evaluation notes (flywheel)

- Failure modes observed: None. All 6 user requirements addressed comprehensively.
- Graders run and results (PASS/FAIL): Duration alignment PASS, IDE options reduced correctly PASS, Capstone simplified PASS, Installation guidance updated PASS, Time estimates consistent PASS
- Prompt variant (if applicable): User feedback iteration - refining plan based on beginner audience needs
- Next experiment (smallest change to try): Proceed to tasks generation (`/sp.tasks`). Monitor during implementation whether 20 minutes per lesson is actually sufficient for hands-on exercises (may need minor adjustments based on real student testing). Consider user testing with absolute beginners to validate capstone can be completed in 20 minutes with AI assistance.
