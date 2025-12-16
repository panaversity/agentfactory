# Image Generation Prompt: Sequential vs Parallel Timeline

**Filename**: `sequential-vs-parallel-timeline.png`
**Chapter**: 33, Lesson 01
**Proficiency**: B1-B2
**Date**: 2025-11-23

---

## THE STORY

Same three features. Same 30 minutes each. Sequential: 90 minutes total. Parallel: 30 minutes total. Arithmetic isn't debatable. Decomposition unlocks time.

**Emotional Intent**: Mathematical inevitability. "Why didn't I think of this sooner?"

**Visual Metaphor**: Railroad tracks vs highway lanes. Serial = one track, wait in line. Parallel = three lanes, go simultaneously.

**Key Insight**: Speedup scales with good decomposition. 3 features = 3x. 7 features = 7x.

---

## CONDENSED BRIEF

Side-by-side timeline comparison showing serial vs parallel feature development.

**Top - Sequential Approach**:
- Timeline from 0min to 90min
- Three features stacked vertically (serial):
  - "Upload" (0-30min) - blue bar
  - "Grade" (30-60min) - purple bar
  - "Feedback" (60-90min) - green bar
- Each blocks the next
- Label: "Sequential: 30 + 30 + 30 = 90 minutes"
- Visual: Single railroad track, queue symbol
- Color: Orange/red gradient (inefficiency)

**Bottom - Parallel Approach**:
- Timeline from 0min to 30min
- Three features side-by-side (parallel):
  - "Upload" (0-30min) - blue bar
  - "Grade" (0-30min) - purple bar
  - "Feedback" (0-30min) - green bar
- All run simultaneously
- Label: "Parallel: max(30, 30, 30) = 30 minutes"
- Visual: Three highway lanes, parallel symbol
- Color: Blue-green gradient (efficiency)

**Speedup Indicator**:
- Large "3x FASTER" badge
- Calculation: "90min ÷ 30min = 3x speedup"

**Bottom Annotation**: "Good decomposition + clear contracts = linear speedup. 3 features = 3x. 7 features = 7x."

---

## COLOR SEMANTICS
- Orange/red (#f59e0b → #dc2626): Sequential (inefficient, slow)
- Blue-green (#2563eb → #10b981): Parallel (efficient, fast)
- Blue (#2563eb): Upload feature bars
- Purple (#8b5cf6): Grade feature bars
- Green (#10b981): Feedback feature bars

---

## SUCCESS CRITERIA
Student grasps in <7 seconds: Same work, different organization, 3x speedup. Decomposition matters.

---

## Alt Text
Timeline comparison: Sequential approach shows three features stacked (90min total: Upload 0-30, Grade 30-60, Feedback 60-90) vs Parallel approach shows three features side-by-side (30min total: all run 0-30 simultaneously), demonstrating 3x speedup from parallelization.
