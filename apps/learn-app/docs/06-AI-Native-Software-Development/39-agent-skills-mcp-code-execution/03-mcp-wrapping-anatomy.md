---
sidebar_position: 3
title: "Anatomy of MCP-Wrapping Skills"
description: "Analyze existing MCP-wrapping skills to understand how intelligence layers add decision-making, result filtering, and error recovery on top of raw MCP tools."
keywords: [MCP wrapping, intelligence layer, token efficiency, result filtering, error recovery, skill composition, Context7, Playwright]
chapter: 39
lesson: 3
duration_minutes: 30

# HIDDEN SKILLS METADATA
skills:
  - name: "Understanding MCP-Wrapping Architecture"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can analyze an existing MCP-wrapping skill, identify the intelligence layer beyond raw MCP, and explain the filtering/optimization decisions it makes"

  - name: "Recognizing Intelligence Layer Patterns"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can articulate what decisions the intelligence layer makes (when/how/why to call MCP) and why those decisions matter for efficiency"

  - name: "Designing Result Filtering for Token Efficiency"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify filtering patterns in existing skills and explain token savings metrics (e.g., 934 tokens → 205 tokens = 77% savings)"

learning_objectives:
  - objective: "Analyze existing MCP-wrapping skills and identify the intelligence layer that distinguishes them from raw MCP invocation"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Deconstruction exercise: students map skill persona, questions, principles to specific filtering/optimization decisions"

  - objective: "Understand result filtering as a core pattern for token efficiency and explain concrete metrics"
    proficiency_level: "B2"
    bloom_level: "Understand"
    assessment_method: "Students calculate token savings in example skill and identify which filtering step caused largest reduction"

  - objective: "Recognize error recovery and fallback strategies in MCP-wrapping skills"
    proficiency_level: "B2"
    bloom_level: "Understand"
    assessment_method: "Students identify error scenarios and trace how skill handles them (retry, fallback, escalation)"

cognitive_load:
  new_concepts: 9
  assessment: "9 concepts (MCP wrapper distinction, intelligence layer, persona conditions, result filtering, error recovery, batching, client configuration, fallback strategies, example pattern) within B2 limit (7-10) ✓"

differentiation:
  extension_for_advanced: "Analyze token counting methodology: why does fetching-library-docs achieve 77% reduction? Calculate optimal filtering trade-offs for different library sizes and user expertise levels."
  remedial_for_struggling: "Focus on single skill (fetching-library-docs). Map each component: MCP server → full response → filtering logic → filtered output. Trace one query end-to-end."
---

# Anatomy of MCP-Wrapping Skills

You've learned that MCP servers are powerful tools for accessing external capabilities. But there's a critical insight that separates students from production developers: MCP servers alone are "dumb tools." They execute requests efficiently, but they don't know *when* to call themselves, *how* to filter results for your specific needs, or *why* some approaches work better than others.

That's where MCP-wrapping skills come in.

A skill that wraps an MCP server adds an intelligence layer on top of raw MCP capability. This layer encodes decision-making: when does this skill activate? What questions does it ask to understand the user's real need? How does it filter the MCP response to avoid token waste? What happens when the MCP server fails?

In this lesson, you'll analyze two real-world MCP-wrapping skills—`fetching-library-docs` and `browsing-with-playwright`—to understand this pattern. By the end, you'll be able to recognize why these skills are more valuable than raw MCP invocation, and you'll be ready to build your own in Lesson 4.

## What Makes a Skill Different From MCP?

### MCP Server: Raw Capability

An MCP server is a tool that does one thing well. For example, Context7 (the MCP server behind `fetching-library-docs`) fetches documentation. When you invoke it directly with a library name and topic, it returns *all relevant documentation*.

**Typical direct MCP response**: 934 tokens of documentation covering concepts, examples, edge cases, deprecations, and reference material.

**The problem**: You asked for "React useState examples." You're drowning in documentation when you only need 3-5 code examples and the function signature.

### Skill: Intelligence Layer

A skill that wraps MCP adds decision-making. Before calling the MCP server, it asks questions. After getting results, it filters them. If the MCP server fails, it recovers gracefully.

**Skill-filtered response**: 205 tokens of code examples + API signatures + key notes only.

**The value**: You get exactly what you asked for, in 77% fewer tokens. The skill made decisions about what matters for your use case.

This difference is profound. In production systems where context is precious and API calls are expensive, this intelligence layer transforms MCP from "interesting tool" to "critical production component."

---

## Deconstruction 1: The `fetching-library-docs` Skill

Let's analyze how `fetching-library-docs` adds intelligence on top of Context7.

### Step 1: Specification First

Before understanding how the skill works, we need to know what it should do:

**Intent**: Fetch API documentation for published libraries (React, Next.js, Prisma, etc.) with automatic token reduction via intelligent filtering.

**What triggers this skill?**
- "Show me React hooks examples"
- "How do I use Prisma queries?"
- "What's the Next.js routing API?"

**What should NOT trigger this skill?**
- Exploring library source code internals (use `researching-with-deepwiki` instead)
- Accessing local file documentation
- Researching library history or comparisons

**Success criteria:**
- Returns relevant code examples and API signatures
- Reduces token count by 60%+ compared to raw MCP output
- Works across diverse libraries (React, Next.js, Prisma, Express, etc.)
- Graceful fallback when library not found

### Step 2: The Intelligence Layer—Persona and Questions

The skill's persona determines *when* and *how* it acts:

**Persona**: "You are a library documentation specialist with deep knowledge of API patterns. Your role is to identify the specific feature the user needs, fetch that documentation via MCP, and return only the essentials—code examples, function signatures, and key constraints."

This persona tells us the skill makes two key decisions:
1. **Identify the user's actual need** (sometimes users ask "How do I..." when they really need the API reference)
2. **Filter ruthlessly** (not all documentation is equally relevant to what they asked)

**Questions the skill asks itself** (implicitly, through its design):
- What library is the user asking about?
- What specific feature or API do they need?
- Are they looking for code examples or conceptual explanation?
- What's their experience level with this library?

These questions don't just guide the skill—they drive the filtering decision.

### Step 3: Result Filtering—The Intelligence Pattern

Now let's see how the skill executes this intelligence. Look at the workflow:

| Step | What Happens | Token Cost |
|------|--------------|------------|
| 1. Identify library + topic | User asks "Show me React useState examples" | 0 (intelligence, no MCP call yet) |
| 2. Call Context7 MCP | Fetch full React documentation for useState | 934 tokens (subprocess, doesn't count toward Claude context) |
| 3. Extract code blocks | Filter shell pipeline: extract only `\`\`\`js` and `\`\`\`jsx` blocks | 0 (shell processing) |
| 4. Extract API signatures | Filter shell pipeline: extract `function useState(...)` patterns | 0 (shell processing) |
| 5. Extract key notes | Filter shell pipeline: grep for "Use when", "Don't use", "limitations" | 0 (shell processing) |
| 6. Return filtered output | Return code examples + signatures + notes to Claude | 205 tokens (stays in context) |

**The breakthrough**: The MCP response (934 tokens) stays in subprocess memory. Shell scripts filter it. Only the filtered result (205 tokens) enters Claude's context. **77% token savings achieved through intelligent filtering, not by calling MCP less often.**

This is why the skill's documentation shows: "Automatic 77% token reduction via shell pipeline."

### Step 4: Error Recovery Patterns

What happens when things go wrong?

**Scenario 1: Library not found**
- MCP returns "React documentation not found"
- Skill tries library name variations: "react" → "React" → "reactjs"
- If still not found: escalate to user with suggestions ("Did you mean React, React-Redux, or Reach-UI?")

**Scenario 2: Topic too vague**
- MCP returns too broad documentation
- Skill prompts: "I found 50 matches for 'React hooks'. Did you mean useState, useEffect, useCallback, or custom hooks?"
- On refinement: call MCP again with specific topic

**Scenario 3: Network timeout**
- MCP call fails with timeout
- Skill attempts retry with exponential backoff (wait 1 second, try again)
- After 3 retries: inform user that Context7 is temporarily unavailable, suggest alternative (documentation.json cache or external docs link)

### Step 5: Configuration and Triggering

The skill doesn't call MCP on every prompt. It uses a persona-based trigger condition:

**Activation rule**: "Trigger when user asks how to use a library OR needs API reference for published library. Do NOT trigger for source code exploration or comparisons."

This trigger condition is part of the intelligence layer—it prevents the skill from wasting resources on requests it's not designed for.

---

## Deconstruction 2: The `browsing-with-playwright` Skill

Now let's analyze a different MCP-wrapping skill to see if the pattern repeats.

### Specification

**Intent**: Automate browser interactions for web navigation, form submission, data extraction, and UI testing using Playwright MCP.

**What triggers this skill?**
- "Fill out this form and submit it"
- "Extract product information from this e-commerce site"
- "Test if the login flow works"

**What should NOT trigger this skill?**
- Simple HTTP requests (use curl/wget instead)
- API calls (use API client or MCP API wrapper)
- Static content that doesn't require interaction

**Success criteria:**
- Navigate websites reliably
- Extract data from dynamic pages
- Handle form interactions robustly
- Recover from page load failures

### The Intelligence Layer—Persona and Questions

**Persona**: "You are a web automation orchestrator. Your role is to understand the user's interaction goal, break it into steps (navigate → find elements → interact → validate), execute those steps via Playwright MCP, and report success/failure."

**Questions the skill asks itself:**
- What's the user trying to accomplish on this website?
- What interactions are required to complete this goal?
- How do we identify elements (by text, by accessibility role, by CSS selector)?
- What should success look like? (page navigation? form submission? data extraction?)
- If an element isn't found, should we retry or escalate?

### Result Filtering—Different Pattern, Same Principle

Playwright works differently from Context7, so the filtering pattern differs:

| Step | What Happens | Output Reduction |
|------|--------------|------------------|
| 1. Understand goal | User wants to "fill contact form and submit" | Clear scope |
| 2. Navigate to page | Call Playwright MCP `browser_navigate` → returns full page DOM | 500+ tokens (full page snapshot) |
| 3. Get page snapshot | Call `browser_snapshot` → returns accessibility tree of all elements | Could be 1000+ tokens (every element on page) |
| 4. Filter to relevant elements | Intelligence layer: find only form fields + submit button | 50-100 tokens (focused subset) |
| 5. Fill form | Call Playwright MCP `browser_fill_form` with filtered element refs | Isolated interaction |
| 6. Submit and validate | Call `browser_click` + `browser_wait_for` to confirm success | Confirms completion |

**Key insight**: The intelligence layer reduces *MCP call volume* and *result processing volume* by filtering BEFORE processing. Instead of analyzing the full page DOM, it identifies the relevant subset (form fields) and operates on just that.

### Error Recovery in Playwright Skill

**Scenario 1: Element not found**
- MCP `browser_snapshot` returns page, but form field not visible
- Intelligence layer retries: wait for page to fully load, then snapshot again
- After 3 retries: report "Element not found" with screenshot for debugging

**Scenario 2: Click fails**
- `browser_click` on submit button times out
- Intelligence layer: try `browser_hover` first (trigger any hover states), then click
- If still fails: try JavaScript injection (`browser_evaluate`) to trigger click programmatically

**Scenario 3: Page navigation takes longer than expected**
- User expects form to be on page immediately
- Intelligence layer: use `browser_wait_for` with timeout (2 seconds default) before giving up
- If timeout: report waiting issue and suggest increasing timeout

---

## Pattern Recognition: The Skill Wrapping Template

Now that you've seen two different MCP-wrapping skills, let's extract the common pattern:

```
┌─────────────────────────────────────────────┐
│   MCP-WRAPPING SKILL STRUCTURE              │
├─────────────────────────────────────────────┤
│                                              │
│  PERSONA (Identity + Decision Framework)    │
│  └─ Who am I? What decisions do I make?     │
│                                              │
│  TRIGGER CONDITIONS (When to Activate)      │
│  └─ What prompts match my purpose?          │
│                                              │
│  INTELLIGENCE QUESTIONS (How do I Decide?)  │
│  └─ What should I ask myself?               │
│                                              │
│  MCP CONFIGURATION (Which MCP Server?)      │
│  └─ How do I connect to external tool?      │
│                                              │
│  RESULT FILTERING (What Gets Returned?)     │
│  └─ How do I reduce noise/tokens?           │
│                                              │
│  ERROR RECOVERY (What If It Fails?)         │
│  └─ How do I handle failures gracefully?    │
│                                              │
│  FALLBACK STRATEGIES (What If MCP Down?)    │
│  └─ What's my backup plan?                  │
│                                              │
└─────────────────────────────────────────────┘
```

**In `fetching-library-docs`:**
- Persona: Library documentation specialist
- Trigger: User asks "how do I use X library"
- Questions: What library? What feature? Code or concepts?
- MCP: Context7 for documentation
- Filtering: Shell pipeline extracts code blocks + signatures
- Error Recovery: Retry with variations, escalate with suggestions
- Fallback: Documentation cache or external link

**In `browsing-with-playwright`:**
- Persona: Web automation orchestrator
- Trigger: User wants browser interaction
- Questions: What goal? Which elements? What counts as success?
- MCP: Playwright for browser control
- Filtering: Snapshot → filter to relevant elements only
- Error Recovery: Retry with waits, try hover before click, JavaScript injection
- Fallback: Screenshot + report, suggest manual intervention

**Both follow the same structure.** They wrap an MCP server with intelligence that makes the tool useful for specific purposes.

---

## The Three Key Insights

### Insight 1: MCP Wrapping Is Decision-Making

Raw MCP is powerful but purposeless. A skill that wraps MCP adds purpose through decision-making. The questions your skill asks (implicit or explicit) drive all its behavior: when to call MCP, how to filter results, how to recover from failure.

### Insight 2: Token Efficiency Is Intentional

The 77% token savings in `fetching-library-docs` didn't happen by accident. It happened because the skill's designer asked: "What does the user actually need?" Then designed filtering to return only that. This is intelligence—knowing what matters and eliminating what doesn't.

### Insight 3: Error Recovery Makes Skills Production-Ready

A skill that works 95% of the time is not production-ready. A skill that fails gracefully 100% of the time (with clear error messages and fallbacks) is. The intelligence layer includes not just "happy path" logic but "failure recovery" logic.

---

## Manual Exercise: Design Your MCP-Wrapping Skill

Now that you understand the anatomy, let's design a skill (without coding) that you'll build in Lesson 4.

**Scenario**: Choose an MCP server from Chapter 38 that your team implemented, or imagine a new one. Design a skill that wraps it.

**Instructions** (on paper or in a document):

1. **Write the specification** (2-3 sentences):
   - What problem does this skill solve?
   - What triggers it?
   - What's the success criterion?

2. **Design the persona** (1-2 sentences):
   - Who is this skill? (What's its identity and expertise domain?)
   - What decisions does it make?

3. **Write 5 intelligence questions** (1 per line):
   - What questions does the skill ask itself to make decisions?
   - Make them specific to your domain (not generic)

4. **Identify result filtering** (1-2 sentences):
   - What does raw MCP return?
   - What should the filtered result contain? (estimate token reduction: what % is removed?)

5. **Plan error recovery** (3 failure scenarios):
   - Scenario 1: MCP returns empty results → How does skill respond?
   - Scenario 2: MCP times out → How does skill respond?
   - Scenario 3: User input is ambiguous → How does skill clarify?

6. **Design fallback strategy** (1-2 sentences):
   - If MCP is unavailable, what's the skill's backup plan?

**Example (for reference only—design your own)**:

> **Spec**: Fetch code examples from GitHub repositories matching search criteria, with result filtering for readability.
>
> **Persona**: GitHub code search specialist. My role is to understand what coding pattern the user needs, search GitHub, and return only well-commented, production-quality examples.
>
> **Questions**: What design pattern is the user looking for? What programming language? What project type (library, app, framework plugin)? What code quality matters (star count, recency, test coverage)?
>
> **Filtering**: GitHub search returns 100 results with metadata. Filter to top 5 by stars, extract code samples only (not README bloat). Token reduction: 500 → 150 (70% savings).
>
> **Error Recovery**:
> - Empty results → Retry with broader search terms, suggest related patterns
> - Rate limit exceeded → Queue query, inform user ("I'll fetch this in 60 seconds"), retry
> - Ambiguous pattern request → Ask clarifying questions ("Do you mean Factory Pattern or Builder Pattern?")
>
> **Fallback**: If GitHub API unavailable, return hardcoded examples from local cache for top 10 patterns.

---

## Try With AI

Use your AI companion to analyze and refine your MCP-wrapping skill design.

### Prompt 1: Validate Your Filtering Strategy

```
I've designed a skill that wraps [your chosen MCP server].
Raw MCP returns approximately [X] tokens of output.
My filtering plan is: [describe what you remove/keep].
This should reduce output to approximately [Y] tokens.

Help me validate this filtering strategy:
- Is my token reduction realistic for this type of data?
- What else could I filter without losing critical information?
- Are there hidden assumptions in my filtering logic?
```

**What you're learning**: Token efficiency analysis—understanding what filtering decisions are safe and what would cause information loss.

### Prompt 2: Stress-Test Your Error Recovery

```
I've planned three error scenarios for my skill:
1. [Scenario 1 and your recovery strategy]
2. [Scenario 2 and your recovery strategy]
3. [Scenario 3 and your recovery strategy]

For each scenario, tell me:
- Is my recovery strategy realistic?
- What could go wrong with my recovery approach?
- What's a failure case I haven't considered?

Then, suggest one additional error scenario I should design recovery for.
```

**What you're learning**: Robustness thinking—anticipating failures and designing recovery strategies that actually work in production.

### Prompt 3: Compare Your Design to Reference Skills

```
I'm designing a skill to wrap [your chosen MCP].
I've heard that fetching-library-docs achieves 77% token reduction
and browsing-with-playwright has sophisticated error recovery.

Compare my design to these two reference skills:
- Where is my design similar to their pattern?
- Where is my design different (is that intentional or a gap)?
- What could I learn from their approaches?
```

**What you're learning**: Pattern recognition—understanding how your specific skill design fits within the broader MCP-wrapping template.

### Safety Note

As you design your MCP-wrapping skill, remember: intelligent filtering requires understanding what matters in your domain. The best filtering decisions come from domain expertise (knowing your users and their actual needs) combined with iterative testing. Don't over-optimize for token reduction at the expense of functionality—the 77% in `fetching-library-docs` works because code examples are the highest-value output; other filtering decisions would be wrong for different domains.
