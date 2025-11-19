---
title: "Clarify Phase - Refining Specs with /sp.clarify"
chapter: 31
lesson: 5
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Using /sp.clarify Command"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can run /sp.clarify on existing specification and interpret feedback"

  - name: "Identifying Specification Gaps"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student recognizes ambiguities, missing assumptions, incomplete requirements in specifications"

  - name: "Iterative Specification Refinement"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student revises specification based on AI feedback and decides which suggestions to accept"

  - name: "Cascade Quality Improvement"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student understands how improved specifications lead to better plans"

  - name: "AI Feedback Interpretation"
    proficiency_level: "B1"
    category: "Soft"
    bloom_level: "Analyze"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student evaluates AI suggestions critically and decides which feedback to implement"

learning_objectives:
  - objective: "Use /sp.clarify command to analyze calculator specification for gaps and ambiguities"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful execution of /sp.clarify and interpretation of feedback"

  - objective: "Identify and resolve specification ambiguities discovered by AI analysis"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Refined specification addressing all identified gaps"

  - objective: "Understand iterative refinement process: specification → AI feedback → revision → improvement"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of how clarification process improved specification"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (Clarify command, gap identification, ambiguity resolution, AI feedback interpretation, iterative refinement, cascade improvement, human decision-making) within B1 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Run /sp.clarify multiple times; document how specification evolves with each iteration; analyze what kinds of gaps AI discovers"
  remedial_for_struggling: "Focus on top 3-4 clarifying questions from AI; resolve those before moving to planning"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/10-chapter-33-redesign/spec.md"
created: "2025-11-05"
last_modified: "2025-11-05"
git_author: "Claude Code"
workflow: "manual-implementation"
version: "1.0.0"
---

# Clarify Phase - Refining Specs with /sp.clarify

You've written a calculator specification. It looks good. But there are always gaps you didn't catch-ambiguities that seemed clear in your head but are actually unclear on paper. Edge cases you missed. Assumptions you didn't state explicitly.

This is where the `/sp.clarify` command helps. **Clarify is NOT a formal workflow phase** like `/sp.specify` or `/sp.plan`. It's a **quick check** that your specification is complete before moving to planning.

Think of `/sp.clarify` as your AI companion putting on a "devil's advocate" hat and asking: "Wait, what about...?" It finds gaps you might have missed, then you decide whether to update your spec.

The goal: Make your specification **so clear** that the planning phase can generate a perfect plan.

---

## What Does /sp.clarify Do?

### The Clarify Command

`/sp.clarify` analyzes your specification and reports:

1. **Ambiguous Terms** - Words that could mean multiple things

   - Example: "should handle errors" (what errors? how should they be handled?)

2. **Missing Assumptions** - Things you assumed but didn't state

   - Example: You assumed division by zero is an error, but didn't state what exception type
   - Example: You assumed float precision, but didn't specify the Python precision limits

3. **Incomplete Requirements** - Operations or cases you didn't cover

   - Example: You specified divide(10, 2) but didn't specify divide(10, 0.0000001)
   - Example: You specified all operations individually but didn't specify operation chaining

4. **Constraint Conflicts** - Where spec might contradict Constitution

   - Example: Spec says "results are exact integers" but Constitution says "always return float"

5. **Edge Case Gaps** - Cases that should be handled but aren't documented
   - Example: What happens with power(2, 10000)? Overflow?
   - Example: What happens with very small numbers (e.g., 0.000000001)?

#### 💬 AI Colearning Prompt
> "Why is /sp.clarify a 'quick check' rather than a formal phase like /sp.specify or /sp.plan? What's the purpose of finding gaps before moving to planning?"

#### 🎓 Expert Insight
> In AI-native development, iterative refinement is faster than "perfect on first try." The Clarify phase embodies this: write a good-enough spec quickly, then use AI to spot gaps you missed. This is MORE efficient than trying to write a perfect spec alone—AI sees patterns humans overlook (missing edge cases, ambiguous terms, Constitution conflicts). Two rounds of clarification produce better specs than hours of solo perfectionism.

---

## Part B: The Clarify Workflow

Here's how clarification works in practice.

### Step 1: Run /sp.clarify

In Claude Code, from your calculator-project directory:

```
/sp.clarify

My calculator specification is at specs/calculator/spec.md
Please analyze it for:
1. Ambiguous terms or vague language
2. Missing assumptions
3. Incomplete requirements
4. Edge cases not covered
5. Any conflicts with my Constitution

What gaps should I address before planning?
```

Your companion will analyze your specification, identify gaps or ambiguities, and ask clarifying questions. Review its findings and update your specification accordingly.

### Step 2: Re-Run /sp.clarify (Optional)

If you made significant changes, run `/sp.clarify` again:

```
I've updated my specification based on your feedback.
Please analyze it again for remaining gaps.
Should I proceed to planning, or are there more clarifications needed?
```

Most specifications need 1-2 clarification rounds. After that, they're ready for planning.

---

## Clarify Your Calculator Specification

Now let's clarify YOUR calculator specification.

### Step 1: Run /sp.clarify on Your Specification

In Claude Code, from your calculator-project directory, run:

```
/sp.clarify

My calculator specification is at specs/calculator/spec.md

Please analyze it for:
1. Ambiguous terms (words that could mean different things)
2. Missing assumptions (things I might have assumed but didn't state)
3. Incomplete requirements (operations or cases not fully specified)
4. Edge cases not covered
5. Conflicts with my project Constitution

List any gaps or questions. Which ones should I address before planning?
```

### Step 2: Verify Readiness

Ask your AI companion:

```
Is my specification now ready for the planning phase?
Or are there critical gaps I should address first?
```

---

## Cascade Quality Validation

Now test the cascade effect: Does your clarified specification improve planning potential?

**Ask yourself**:

- ✅ Can a developer read your spec and understand exactly what to build?
- ✅ Are all edge cases documented?
- ✅ Are error behaviors explicit (which exception, which message)?
- ✅ Is precision defined?
- ✅ Are operation interfaces clear?

If yes to all, your spec is ready for planning.

---

## Common Mistakes

### Mistake 1: Skipping /sp.clarify Because "Spec Looks Good to Me"

**The Error**: "I wrote a detailed spec. I don't need clarification."

**Why It's Wrong**: We're blind to our own ambiguities. What's "obvious" to you may be unclear on paper.

**The Fix**: Always run `/sp.clarify`. You'll be surprised what gaps emerge. Most specs need 1-2 clarification rounds.

### Mistake 2: Accepting All AI Suggestions Without Critical Thinking

**The Error**: AI says "Consider adding X" → immediately adding X without evaluating necessity

**Why It's Wrong**: Not all suggestions improve your spec. Some add unnecessary complexity.

**The Fix**: Evaluate each suggestion:
- Is this edge case likely in real use?
- Does this improve clarity or add noise?
- Does this align with my Constitution?

Then decide: Accept, Reject, or Modify.

#### 🤝 Practice Exercise

> **Ask your AI**: "I ran /sp.clarify on my calculator specification and received 5-7 suggestions for improvement. Can you help me prioritize: (1) Which suggestions are CRITICAL (spec won't work without them)? (2) Which are NICE-TO-HAVE (improve quality but not blocking)? (3) Which can I defer to future iterations? Then explain how addressing the critical gaps will improve the planning phase quality."

**Expected Outcome**: Your AI should categorize clarification feedback by urgency (e.g., missing error handling = critical, additional edge cases = nice-to-have), explain why critical gaps block planning (ambiguous requirements → ambiguous plans), and help you make informed decisions about which changes to prioritize before moving to /sp.plan.

---

## Try With AI

Ready to validate your clarified specification and ensure it's complete enough for planning? Test your improvements:

**🔍 Explore Ambiguity Detection:**
> "Compare my original calculator spec with the clarified version at `specs/calculator/spec.md`. What ambiguities were resolved? What gaps were filled? Show me specific examples where clarification improved implementation readiness. Are there still any vague requirements remaining?"

**🎯 Practice Implementation Readiness:**
> "Imagine you're a developer who's never seen this project before. Read my clarified calculator specification and tell me: (1) Can you implement all 5 operations from this spec alone? (2) What questions would you still need to ask me? (3) What edge cases or error conditions are unclear? (4) Is the specification complete enough to write tests before code?"

**🧪 Test Prioritization Decisions:**
> "I received 7 clarification suggestions from `/sp.clarify`: [list your suggestions]. Help me prioritize: Which are CRITICAL (spec won't work without them)? Which are NICE-TO-HAVE (improve quality but not blocking)? Which can I defer to later? Explain your reasoning for each category."

**🚀 Apply to Your Spec:**
> "I just ran `/sp.clarify` on my [describe your feature] specification. I got feedback about [describe feedback areas]. Help me decide which clarifications to address now vs later. What's the minimum set of changes needed to make my spec ready for the planning phase? Walk me through your decision framework."

---
