# Image Generation: Lessons Learned & Improved Workflow

**Date**: 2025-01-12
**Context**: Chapter 2 Paradigm Shift image regeneration struggle (5+ iterations)

---

## What Went Wrong

### The Struggle Timeline
1. **Iteration 1**: Fixed spelling "Orchestrated" → still had issues
2. **Iteration 2**: Fixed capitalization "autonomously" → flow diagram cluttered
3. **Iteration 3**: Simplified flow → still had "ORCHESTRATED ACTIONS" in caps
4. **Iteration 4**: Better design → user said "too bad concept"
5. **Iteration 5**: Finally removed "Orchestrated" entirely → success

### Root Causes
1. **No user validation before generation** - Assumed prompt was correct
2. **Complex prompts** - More instructions = more confusion for AI
3. **Reactive fixes** - Fixed what was wrong vs. designing what was right
4. **Missing clarity checkpoint** - Didn't ask "what's the simplest version?"

---

## New Workflow: SPEC → VALIDATE → GENERATE

### Phase 1: UNDERSTAND (Don't skip this!)

**Before writing any prompt, ask:**

1. **What is the ONE core message of this image?**
   - Example: "Old way: navigate interfaces. New way: state intent."

2. **What's the MINIMUM visual needed to convey it?**
   - Example: "Two side-by-side flow diagrams"

3. **What text is ESSENTIAL vs. NICE-TO-HAVE?**
   - Essential: Flow labels (User → Interface → Action)
   - Nice-to-have: Explanatory bullets (can be removed if cluttered)

4. **Ask user for validation:**
   ```
   "Before I generate, let me confirm: You want a simple split-screen showing:
   - Left: User → Interface → Action (old way)
   - Right: User Intent → Agent → Actions (new way)
   - Minimal text only

   Is this the right concept?"
   ```

### Phase 2: WRITE MINIMAL PROMPT

**Prompt Structure:**

```markdown
[IMAGE TYPE] [ASPECT RATIO] [BACKGROUND]

LEFT SECTION:
- Title: "[SIMPLE TITLE]"
- Flow: [BOX 1] → [BOX 2] → [BOX 3]
- Bullets (max 2): • [ESSENTIAL POINT 1] • [ESSENTIAL POINT 2]

RIGHT SECTION:
- Title: "[SIMPLE TITLE]"
- Flow: [BOX 1] → [BOX 2] → [BOX 3]
- Bullets (max 2): • [ESSENTIAL POINT 1] • [ESSENTIAL POINT 2]

CRITICAL (negative instructions):
- NO extra labels in boxes
- NO additional text beyond what's specified
- Keep it SIMPLE

Style: Clean, minimal, professional
```

**Key Principles:**
- ✅ Start with 50% of the detail you think you need
- ✅ Use NEGATIVE instructions ("NO extra text") not just positive
- ✅ Specify exact text for EVERY element
- ✅ Keep total prompt under 300 words

### Phase 3: GENERATE → REVIEW → DECIDE

After generation:

1. **Review with user FIRST** before assuming it's correct
2. **Ask**: "Does this match your vision? What needs to change?"
3. **If changes needed**: Go back to Phase 1 (understand the gap)

---

## Specific Improvements for Skills

### 1. **visual-asset-workflow Skill Update**

**Current problem**: Generates verbose prompts with too much detail

**Improvement**: Add "PROMPT MINIMALISM CHECK"

```markdown
Before finalizing prompt, ask:
1. Can any text be removed without losing the core message?
2. Are there any ambiguous terms that might confuse image generator?
3. Have I included NEGATIVE instructions for what NOT to do?
4. Is the prompt under 300 words?
```

### 2. **Image Generation Agent Enhancement**

**New step**: PRE-GENERATION VALIDATION

When user requests image regeneration:

```markdown
1. Read existing image
2. Read lesson content to understand context
3. DRAFT minimal prompt
4. ASK USER: "Before generating, here's my understanding:
   - Core message: [X]
   - Visual approach: [Y]
   - Text elements: [Z]
   Does this match your vision?"
5. ONLY AFTER APPROVAL → Generate
```

### 3. **Prompt Template Library**

Create reusable minimal prompts:

**Template: Split-Screen Comparison**
```
Clean split-screen comparison 16:9 white background.

LEFT SIDE ([color]):
Title: "[Old Paradigm Name]"
Flow: [Step 1] → [Step 2] → [Step 3]
• [Characteristic 1]
• [Characteristic 2]

CENTER: Large [color] arrow with "[TRANSITION WORD]"

RIGHT SIDE ([color]):
Title: "[New Paradigm Name]"
Flow: [Step 1] → [Step 2] → [Step 3]
• [Characteristic 1]
• [Characteristic 2]

CRITICAL:
- NO extra text in flow boxes
- Keep labels to 1-2 words max
- Clean readable design
```

**Template: Process Diagram**
```
[Layout type] 16:9 [background color].

Title: "[Main Concept]"

[Number] [shapes] arranged [pattern]:

[Shape 1]:
- Icon: [description]
- Label: "[1-2 words]"

[Repeat for each element]

CRITICAL:
- NO descriptions beyond label
- NO extra decorative text
- Minimal clean style
```

---

## Decision Framework: When to Push Back

**ASK USER TO CLARIFY when:**

1. Feedback is vague ("bad design", "looks wrong")
   - Response: "What specifically should change? Color? Layout? Text amount?"

2. Requirements seem contradictory
   - Response: "You mentioned X but also Y. Which is higher priority?"

3. Text overload detected (>5 items per section)
   - Response: "This feels dense. Can we reduce to the 3 most important points?"

4. Concept unclear
   - Response: "Let me sketch the concept in text first. Here's what I understand: [X]. Is this right?"

---

## Metrics for Success

**Good image generation workflow:**
- ✅ User validates concept BEFORE generation (not after)
- ✅ First generation is 80%+ correct
- ✅ Maximum 2 iterations to final version
- ✅ Prompt is under 300 words
- ✅ No spelling errors (due to minimal text)

**Bad image generation workflow:**
- ❌ Generate first, validate later
- ❌ First attempt needs major changes
- ❌ 3+ iterations to get it right
- ❌ Prompt is verbose (500+ words)
- ❌ Spelling errors slip through

---

## Action Items

### Immediate (Next Image Request)
1. [ ] Ask clarifying questions BEFORE writing prompt
2. [ ] Draft prompt with 50% less detail than instinct says
3. [ ] Get user validation on concept before generating
4. [ ] Use negative instructions ("NO extra text")

### Short-term (Next Chapter)
1. [ ] Create prompt template library (5 common image types)
2. [ ] Update visual-asset-workflow skill with minimalism check
3. [ ] Add pre-generation validation step to workflow

### Long-term (Project-wide)
1. [ ] Document "Image Generation Best Practices" in constitution
2. [ ] Create image style guide (what "minimal" means for this project)
3. [ ] Build prompt validation checklist

---

## Examples: Before & After

### BEFORE (What I Did Wrong)
```
Create a professional comparison diagram showing the fundamental shift from
User Interface to User Intent interaction paradigm.

Layout: Split-screen comparison with central arrow transition, 1792x1024px (16:9 aspect ratio).
- Background: White (#FFFFFF)
- Left panel (Old Paradigm): 800px wide, Light Gray tint (#F8F9FA)
- Center transition: 192px wide with large right arrow, Orange gradient
- Right panel (New Paradigm): 800px wide, Light Orange tint (#FFF8F5)
- Each panel has 48px internal padding

[...300 more words of detailed specifications...]
```
**Result**: Cluttered, wrong text, 5 iterations needed

### AFTER (What I Should Do)
```
Split-screen comparison 16:9 white background.

LEFT (light gray): "Old: User Interface"
User → Interface → Action

RIGHT (light orange): "New: User Intent"
User Intent → Agent → Actions

Center: Orange arrow "SHIFT"

CRITICAL: NO extra text in boxes. Keep it simple.
```
**Expected Result**: Clean, correct on first try

---

## Reflection Questions (Before Every Generation)

1. **Have I asked the user to validate my understanding?**
2. **Could this prompt be 50% shorter without losing meaning?**
3. **Am I overspecifying details the AI will get wrong anyway?**
4. **What's the ONE thing this image must communicate?**
5. **If I only had 100 words, what would I say?**

---

## Conclusion

**Core Insight**: Less is more. Simple prompts generate better images than complex ones.

**New Mantra**:
> "Validate concept → Minimal prompt → Generate → Iterate ONLY if needed"

**Success Metric**: User says "perfect" on first or second attempt, not fifth.
