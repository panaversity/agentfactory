# Image Generation Prompt: Communication Complexity Graph

**Filename**: `communication-complexity-graph.png`
**Chapter**: 33, Lesson 04
**Proficiency**: B2 (Advanced)
**Date**: 2025-11-23

---

## THE STORY

"Let's add two more features—shouldn't be much harder, right?" Wrong. Communication grows as N². Three features = manageable. Seven features = chaos. Ten features = impossible without tooling.

**Emotional Intent**: Scaling awakening. "Oh... this doesn't scale linearly. I need different approaches at different scales."

**Visual Metaphor**: Graph showing exponential growth. Gentle slope → steep cliff.

**Key Insight**: Coordination isn't free. Cost grows faster than team size.

---

## CONDENSED BRIEF

Graph showing how communication complexity grows with team size.

**Graph Structure**:
- X-axis: Number of Features/Agents (1 to 10)
- Y-axis: Integration Points (connections needed)
- Curve showing N×(N-1)/2 growth

**Data Points** (marked on curve):
- 3 features: 3 integration points (green zone - "Manual coordination works")
- 5 features: 10 integration points (yellow zone - "Getting complex")
- 7 features: 21 integration points (orange zone - "Tooling helps")
- 10 features: 45 integration points (red zone - "Tooling required")

**Zone Colors**:
- Green (#10b981): 1-3 features - "Manual coordination scales"
- Yellow (#fbbf24): 4-5 features - "Specs + contracts needed"
- Orange (#f59e0b): 6-7 features - "Orchestration tools help"
- Red (#dc2626): 8+ features - "Automation required"

**Formula Box**: "Integration points = N × (N-1) ÷ 2" (shown with example: 10 features = 10×9÷2 = 45 points)

**Bottom Annotation**: "Why orchestration matters: Complexity grows as N². Tooling turns exponential into linear."

---

## PEDAGOGICAL REASONING
This visual teaches scaling limits—helps students understand WHEN to introduce orchestration tools (not "never" or "always", but "at threshold").

---

## COLOR SEMANTICS
- Green (#10b981): Safe zone (manual works)
- Yellow (#fbbf24): Caution zone (specs help)
- Orange (#f59e0b): Tool zone (orchestration recommended)
- Red (#dc2626): Danger zone (automation required)
- Blue (#2563eb): Curve line, data points

---

## SUCCESS CRITERIA
Student grasps in <12 seconds: Communication grows as N². 3 features = easy. 10 features = impossible without tooling. Threshold around 5-7.

---

## Alt Text
Graph showing communication complexity growth: X-axis (1-10 features), Y-axis (integration points), curve showing N² growth with color-coded zones—green (1-3: manual OK), yellow (4-5: specs needed), orange (6-7: tooling helps), red (8+: automation required). Formula N×(N-1)÷2 explains exponential scaling.
