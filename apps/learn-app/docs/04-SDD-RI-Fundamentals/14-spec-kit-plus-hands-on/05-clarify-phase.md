---
title: "Clarify Phase"
chapter: 14
lesson: 5
duration_minutes: 30
proficiency_level: "B1"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Using /sp.clarify Command"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can run /sp.clarify on specification and interpret gap-detection feedback"

  - name: "Identifying Specification Gaps"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student recognizes ambiguities and missing assumptions in specifications"

  - name: "Iterative Specification Refinement"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student revises specifications based on /sp.clarify feedback and makes refinement decisions"

learning_objectives:
  - objective: "Use /sp.clarify command to identify gaps in specifications"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful execution of /sp.clarify and interpretation of feedback"

  - objective: "Recognize ambiguities and missing assumptions in written specifications"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Refined specification addressing identified gaps"

  - objective: "Iteratively refine specifications based on AI feedback"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Completed clarification cycle with documented refinements"

cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (Clarify workflow, gap detection, ambiguity resolution, iterative refinement) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Run /sp.clarify multiple times; document specification evolution; analyze how each clarification improves plan quality"
  remedial_for_struggling: "Focus on top 2-3 clarifying questions from /sp.clarify; resolve those before moving to planning"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/037-chapter-14-research-paper-pivot/spec.md"
created: "2025-11-26"
last_modified: "2025-11-26"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Clarify Phase

In Lesson 04, you wrote a specification for your research paper. It looked complete. But there are always gaps you didn't catch—ambiguities that seemed clear in your head but are actually vague on paper. Assumptions about scope, audience, or success that you didn't state explicitly.

This is where the `/sp.clarify` command helps. **Clarify is a quick check** that your specification is complete before moving to planning.

Think of `/sp.clarify` as your AI companion putting on a "detail detective" hat and asking: "Wait, who exactly is your audience? What counts as a 'well-researched' paper? How many sources is enough? What format should this follow?" It finds gaps you might have missed, then you decide whether to update your spec.

The goal: Make your specification **so clear** that the planning phase can generate a perfect implementation plan.

---

## What Does /sp.clarify Do?

### The Clarify Command

`/sp.clarify` analyzes your specification and reports:

1. **Ambiguous Terms** - Words that could mean multiple things
   - Example: "well-researched paper" (5 sources? 50 sources? What counts as credible?)
   - Example: "professional format" (APA? MLA? Chicago? Single-spaced? Double-spaced?)

2. **Missing Assumptions** - Things you assumed but didn't state
   - Example: You assumed academic paper but didn't specify citation style
   - Example: You assumed 3,000-word length but didn't state minimum or maximum
   - Example: You assumed English but didn't specify if other languages acceptable

3. **Incomplete Requirements** - Scenarios or cases you didn't cover
   - Example: You specified content but didn't specify editing or revision process
   - Example: You specified research but didn't specify how to handle conflicting sources
   - Example: You specified "introduce topic" but didn't define what introduction contains

4. **Scope Conflicts** - Places where scope is unclear or inconsistent
   - Example: "Comprehensive research" on what exactly? All historical context or just recent developments?
   - Example: "Clear structure" using what organization method? Chronological? Thematic?
   - Example: "Compelling conclusion" appealing to whom? Academic audience? General readers?

### Why Clarify Matters Before Planning

A vague specification creates a vague plan. When the planning phase can't understand exactly what you want, it generates ambiguous design decisions. Then you spend time during implementation realizing your actual intention wasn't captured.

**Without clarification** (vague spec):
```
Intent: Write research paper on climate change
Success Criteria: Paper is well-researched and professionally written
```

Planning phase has questions:
- What specific aspect of climate change? Global warming trends? Policy solutions? Historical development?
- How many sources? Academic only or include journalistic sources?
- What length? 2 pages? 10 pages? 50 pages?
- What citation style? Who's the audience?

**With clarification** (precise spec):
```
Intent: Write research paper on climate policy solutions adopted since 2015
Success Criteria:
- Minimum 5 peer-reviewed sources (journals, not news)
- APA format, 3,000-4,000 words
- Three policy solutions compared (effectiveness, adoption barriers, future outlook)
- Audience: undergraduate economics students
- Conclusion: assessment of which policy approach shows most promise
```

Planning phase now has clear requirements and generates specific implementation tasks.

---

## The Clarify Workflow

### Step 1: Run /sp.clarify

In Claude Code, from your research-paper directory:

```
/sp.clarify

My research paper specification is at specs/paper/spec.md
Please analyze it for:
1. Ambiguous terms (what does "well-researched" mean? How many sources? What type?)
2. Missing assumptions (citation style? audience? paper length? structure?)
3. Incomplete requirements (what does "introduce topic" contain? how to handle conflicting sources? revision process?)
4. Scope conflicts (is this historical overview or current policy analysis? broad or narrowly focused?)

What gaps should I address before planning the paper structure?
```

Your AI companion will analyze your specification, identify gaps or ambiguities, and ask clarifying questions. Review its findings and consider which gaps are critical versus nice-to-have.

### Step 2: Update Your Specification

For each clarifying question, decide: **Do I need to answer this before planning?**

- **Critical gaps** (planning can't work without this): Update spec immediately
  - Example: Citation style is critical (affects all references)
  - Example: Paper length is critical (determines research scope)
  - Example: Audience is critical (determines tone and complexity)

- **Nice-to-have clarifications** (planning can proceed): Update spec or defer
  - Example: Specific revision timeline
  - Example: Preferred formatting tools
  - Example: Aesthetic preferences

### Step 3: Re-Run /sp.clarify (Optional)

If you made significant changes, run `/sp.clarify` again:

```
I've updated my research paper specification based on your feedback.
Please analyze it again for remaining gaps.
Is this specification clear enough to proceed to the planning phase?
```

Most specifications need 1-2 clarification rounds. After that, they're ready for planning.

---

## Clarifying Your Paper Specification

Now let's clarify YOUR research paper specification—the one you wrote in Lesson 04.

### Step 1: Run /sp.clarify on Your Specification

In Claude Code, from your research-paper directory, run:

```
/sp.clarify

My research paper specification is at specs/paper/spec.md

Please analyze it for:

1. AMBIGUOUS TERMS
   - What does "well-researched" mean in my spec? (how many sources? which types?)
   - What does "professional format" mean? (which citation style? spacing? margins?)
   - What does "clear structure" mean? (how many sections? what should each contain?)

2. MISSING ASSUMPTIONS
   - What citation style should I use? (APA, MLA, Chicago, Harvard?)
   - What's the target audience? (academic, general readers, specific field?)
   - What's the paper length? (minimum and maximum word count?)
   - How recent should sources be? (published in last 5 years? 10 years?)

3. INCOMPLETE REQUIREMENTS
   - What should the introduction contain? (background? thesis statement? scope?)
   - How do I handle conflicting sources? (which viewpoints to include?)
   - What constitutes a "credible" source? (peer-reviewed only? news acceptable?)
   - How should I structure the paper? (chronological? thematic? by source?)

4. SCOPE CONFLICTS
   - Is this narrowly focused on one aspect or broadly covering the topic?
   - Is this historical overview or current-state analysis?
   - Are there sub-questions I should address or exclude?

List any gaps or questions. Which ones are CRITICAL (planning won't work without them) vs NICE-TO-HAVE (improve quality but not blocking)?
```

### Step 2: Evaluate Feedback

Review the clarifying questions your AI companion identified. For each one, ask:

- Is this critical to planning the paper structure?
- Can planning proceed without this answer, or does it affect section design?
- Should I resolve this now or defer it?

### Step 3: Update Your Specification

Update your spec.md with the clarifications you decide are critical. You might add:

```
Audience: Undergraduate economics students (not specialized researchers)
Citation Style: APA format, 7th edition
Length: 3,000-3,500 words
Source Types: Peer-reviewed journals (80%), reputable news sources (20%)
Structure: Introduction → Problem Analysis → Three Solutions → Comparison → Conclusion
```

### Step 4: Verify Readiness

Ask your AI companion:

```
Based on the clarifications I've made, is my research paper specification now ready for the planning phase?
Can you explain the paper structure and success criteria back to me to confirm we're aligned?
```

---

## Why Clarification Prevents Implementation Problems

Skipping clarification creates cascading problems during implementation:

**Missing specification clarity** →
**Vague planning decisions** →
**Confused implementation tasks** →
**Rework and frustration**

Here's how clarification breaks this chain:

1. You run `/sp.clarify` and discover ambiguity: "What's the minimum number of sources?"
2. You update spec: "Minimum 6 peer-reviewed sources"
3. Planning phase generates clear implementation task: "Research and select 6+ peer-reviewed sources on [topic]"
4. Implementation proceeds smoothly because the requirement is explicit

---

## Common Mistakes

### Mistake 1: Skipping /sp.clarify Because "Spec Looks Good to Me"

**The Error**: "I wrote a detailed spec. I don't need clarification."

**Why It's Wrong**: Every specification has ambiguities you didn't notice. Clarify surfaces them now (5 minutes) instead of during implementation (5 hours).

**The Fix**: Always run `/sp.clarify`. You'll be surprised what gaps emerge. Most specs need 1-2 clarification rounds.

### Mistake 2: Ignoring Critical Clarifications

**The Error**: "AI asked about citation style but I'll just figure that out later."

**Why It's Wrong**: Citation style affects every source reference. Deferring this decision means planning the paper structure without knowing how citations work, then discovering mid-implementation you chose wrong.

**The Fix**: Address critical gaps upfront. Test: "If planning didn't know this, would they make a different choice?" If yes, it's critical.

### Mistake 3: Accepting All AI Suggestions Without Thinking

**The Error**: AI suggests adding source diversity requirements → immediately adding without evaluating necessity

**Why It's Wrong**: Not all suggestions improve your spec. Some add unnecessary complexity.

**The Fix**: Evaluate each suggestion:
- Is this critical to paper quality or nice-to-have?
- Does this affect planning or just implementation?
- Can I defer this to revision?

Then decide: Accept, Reject, or Modify.

---

## Try With AI

Ready to clarify your research paper specification? Test these prompts:

**Explore Specification Gaps:**
> "I'm ready to run /sp.clarify on my research paper specification. Before I do, what are the most common gaps in paper specifications? What questions should I expect the clarification process to surface?"

**Interpret Clarification Feedback:**
> "Here are the clarifying questions my AI identified about my paper spec: [paste the questions]. Help me categorize them: (1) Which are CRITICAL (planning won't work without this)? (2) Which are NICE-TO-HAVE (improve quality but not blocking)? (3) Which can I defer? Explain how each critical gap affects the planning phase."

**Validate Clarity:**
> "I've updated my research paper specification based on clarification feedback. Read my updated spec and tell me: (1) Is the paper scope clear? (2) Would a planner know what paper structure to design? (3) Are there any remaining ambiguities? (4) Is this specification ready for the planning phase?"

**Practice Decision-Making:**
> "My clarification feedback included suggestions about [topic]. Help me decide whether to address each suggestion now: Is it critical for planning? Does it affect success criteria? Can I defer it to revision? Walk me through your decision framework."

---
