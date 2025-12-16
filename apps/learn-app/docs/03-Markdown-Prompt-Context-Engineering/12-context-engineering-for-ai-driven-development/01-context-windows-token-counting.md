---
title: "Context Windows and Token Counting"
description: "Understand AI context windows and learn manual tracking through session notes and token estimation"
sidebar_label: "Context Windows & Token Counting"
sidebar_position: 1
chapter: 11
lesson: 1
duration_minutes: 40
proficiency: "A2"
concepts: 4

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Context Window Understanding"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student explains context windows as working memory and estimates token usage"

learning_objectives:
  - objective: "Understand context windows as AI working memory with fixed capacity"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Explanation of context window concepts and real-world analogies"

  - objective: "Estimate token usage using word-to-token conversion rules"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Token estimation calculations for sample texts"

  - objective: "Create session notes to track context utilization manually"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Session note document with token tracking"

cognitive_load:
  new_concepts: 4
  assessment: "4 concepts (context windows, tokens, utilization thresholds, session notes) within A2 limit of 5-7 ✓"

differentiation:
  extension_for_advanced: "Research tokenization algorithms; compare context window architectures across different LLMs; analyze how extended context (1M tokens) changes development workflows"
  remedial_for_struggling: "Focus on single concept: 'Context is like computer RAM that fills up'. Practice only token estimation using 1 word = 1 token rule before learning session notes"
---

# Context Windows and Token Counting

You've been working with AI tools like Claude Code for several chapters now. Sometimes responses feel sharp and specific. Other times, responses feel generic or repetitive. Sometimes the AI seems to forget what you discussed earlier in the same session.

These patterns aren't random. They're symptoms of context engineering—or the lack of it.

In this lesson, you'll learn to recognize what "context" means in AI development, estimate how much context you're using, and identify when you're approaching limits. You'll practice tracking context manually through session notes, building the foundation for more advanced context management techniques.

## Understanding Context Windows

When you start a conversation with Claude Code, imagine opening a notebook with exactly 200,000 pages. Everything you type and every response Claude generates gets written in this notebook, page by page. When the notebook fills up, older pages start getting compressed or forgotten to make room for new ones.

That notebook is your **context window**—the AI's working memory for your session.

### Context is Working Memory, Not Long-Term Storage

A context window is NOT:
- Permanent memory (it resets when you start a new session)
- Unlimited capacity (Claude Sonnet 4.5 has 200K tokens standard, 1M extended)
- Shared across sessions (Session A and Session B have separate context)

A context window IS:
- Working memory for the current conversation
- A fixed capacity that fills up as you work
- Shared between everything you load (files, messages, responses)

### Real Context Window Sizes

Different AI tools have different context capacities:

| Tool | Standard Context | Extended Context |
|------|-----------------|------------------|
| **Claude Sonnet 4.5** | 200K tokens | 1M tokens |
| **Gemini 1.5 Pro** | 2M tokens | 2M tokens |

**What this means for you**: In Claude Code, you're typically working with 200,000 tokens of context. That sounds like a lot, but it fills faster than you expect.

### Estimating Context: The Token-to-Word Rule

Since you can't see token counts directly, use this rule of thumb:

**1 word ≈ 1-1.2 tokens**

For quick estimates:
- **Simple text**: Multiply word count by **1.0**
- **Technical content with symbols**: Multiply word count by **1.2**

Example:
```
"Fix authentication bug in user login" = 6 words ≈ 7 tokens
```

## Session Notes: Your Context Tracking Tool

Before you can manage context efficiently, you need to track it. The simplest tool is a **session note**—a Markdown file documenting what you loaded, what you discussed, and how much context you've used.

### Session Note Template

Here's a basic template you'll use for tracking context:

```markdown
# Development Session — [DATE]

## Task: [What you're working on]

## Context Loaded:
- [File 1] — Est. [X] tokens
- [File 2] — Est. [Y] tokens
- Total loaded: ~[Z] tokens

## Progress:
1. [Action taken]
2. [AI response summary]
3. [Your observation]

## Token Estimation:
- Context loaded: ~[X] tokens
- Conversation so far: ~[Y] tokens (count words, multiply by 1.2)
- Total utilization: [X + Y] / 200,000 = [%]

## Observations:
- When did responses change tone or quality?
- When did AI forget earlier context?
- At what utilization % did patterns shift?
```

### Example Session Note

Here's what a real session note might look like:

```markdown
# Development Session — 2025-11-18

## Task: Write user guide for Chapter 11 markdown syntax

## Context Loaded:
- Chapter 11 outline (chapter-09-outline.md) — Est. 800 tokens
- Markdown syntax reference — Est. 1,200 tokens
- Previous chapter examples — Est. 2,000 tokens
- Total loaded: ~4,000 tokens

## Progress:
1. Asked Claude to suggest structure for user guide
2. Claude recommended 5 sections: Introduction, Basic Syntax, Advanced Features, Examples, Common Mistakes
3. I clarified that "Common Mistakes" section isn't needed (minimal content principle)
4. Claude adapted structure to 4 sections, focusing on examples over warnings
5. Drafted Introduction section collaboratively
6. Drafted Basic Syntax section

## Token Estimation:
- Context loaded: ~4,000 tokens
- Conversation (6 exchanges, ~1,500 words): ~1,800 tokens
- Total utilization: 5,800 / 200,000 = 2.9%

## Observations:
- Responses felt specific and well-tailored (low utilization, plenty of room)
- No forgotten context detected
- AI adapted suggestions based on feedback
```

**Observation**: At 2.9% utilization, this session has plenty of room. Responses are specific, AI remembers earlier constraints, and there's no evidence of context degradation.

## Warning Thresholds: Green, Yellow, Red

As your session progresses, context utilization increases. Use these thresholds to guide your actions:

### 🟢 Green Zone (0-70% utilization)
- **Status**: Safe working range
- **Behavior**: Responses are specific, AI remembers context well
- **Action**: Continue working normally

### 🟡 Yellow Zone (70-85% utilization)
- **Status**: Approaching limits
- **Behavior**: Responses may become slightly generic or slower
- **Action**: Plan to create a checkpoint soon

### 🔴 Red Zone (85-100% utilization)
- **Status**: Context degradation likely
- **Behavior**: AI forgets patterns, repeats suggestions, responses become vague
- **Action**: Create checkpoint NOW and restart session

**Why these thresholds?** Research from Google's context engineering team shows that degradation symptoms typically appear around 70-80% utilization, becoming severe above 85%.

## Observable Behaviors vs Token Counts

Since you can't measure token counts precisely (without programming tools), you'll rely on **observable behaviors** to recognize when context is filling up.

### Early Warning Signs (Yellow Zone)

Watch for these patterns:
- AI responses take **slightly longer** to generate
- Responses become **shorter** than earlier in the session
- AI starts asking for **clarification** on things you stated earlier
- Suggestions feel **less specific** to your project

### Critical Symptoms (Red Zone)

These symptoms indicate severe context degradation:
- AI **repeats the same suggestion** multiple times without acknowledgment
- AI **forgets project patterns** stated earlier in the session
- Responses are **generic** ("Use best practices") instead of specific
- AI asks you to **re-explain information** already provided

**You'll explore these symptoms in depth in Lesson 2.**

## Practice: Manual Context Tracking

Now it's time to build the habit of tracking context manually. These exercises help you develop awareness of context usage before using AI to manage it.

### Exercise 1: Write a Session Note

**Task**: Write a session note for a hypothetical development task.

**Scenario**: You're working with Claude Code to write a project README. You've loaded:
- Project specification (2,500 words)
- Existing documentation examples (1,800 words)
- Chapter structure outline (600 words)

You've had 8 exchanges with Claude (approximately 2,000 words total).

**Your Task**:
1. Create a session note using the template above
2. Estimate tokens for each piece of context (use word count × 1.2)
3. Calculate total utilization percentage
4. Identify which warning zone you're in (green, yellow, or red)

### Exercise 2: Compare Two Projects

**Project A**: Small markdown documentation
- 3 files loaded (total 5,000 words)
- 10 exchanges with AI (2,500 words)

**Project B**: Complex specification project
- 12 files loaded (total 18,000 words)
- 15 exchanges with AI (4,000 words)

**Your Task**:
1. Calculate utilization for both projects
2. Which project is closer to yellow zone?
3. Which project should create a checkpoint sooner?
4. Explain your reasoning

### Exercise 3: Identify Warning Signals

Read this conversation excerpt:

```
YOU: "Based on our earlier discussion about using camelCase for variables, how should I name this function?"

AI: "There are many naming conventions. You could use camelCase, snake_case, or PascalCase depending on your project's style guide."

YOU: "But we agreed on camelCase 10 minutes ago."

AI: "You're right, I apologize for the confusion. Let's use camelCase consistently."
```

**Your Task**:
1. Which context degradation symptom is present?
2. If you see this behavior, what utilization range are you likely in?
3. What action should you take?

## Try With AI

Now that you understand context windows and manual estimation, practice validating your estimates with Claude Code.

### Setup
Open Claude Code and keep your session notes nearby for reference.

### Prompt Set

**Prompt 1 — Verify Token Estimate:**
```
I estimated this session note uses approximately [X] tokens. Here's my breakdown:
- [List your calculations]

Looking at the word count and content complexity, is my estimate reasonable? What factors might I have missed?
```

**Prompt 2 — Estimate Utilization for Your Current Session:**
```
I've loaded these files: [list files with word counts]
We've had [N] exchanges with approximately [M] words total.

Help me estimate: What's my current context utilization percentage?
```

**Prompt 3 — Identify Warning Zone:**
```
Based on this utilization estimate: [your calculation]

Which warning zone am I in (green/yellow/red)? What behaviors should I watch for at this utilization level?
```

**Prompt 4 — Plan Checkpoint Timing:**
```
My current utilization is [X%]. I have [Y] more tasks to complete in this session, each requiring approximately [Z] exchanges.

Should I create a checkpoint now, or can I continue? Explain your recommendation.
```

### Expected Outcomes

Claude Code should:
- Validate your token estimates (within ~20% accuracy)
- Explain factors you might have missed (formatting overhead, special characters)
- Confirm warning zone thresholds
- Recommend checkpoint timing based on remaining work

**Safety Note**: Trust your manual observations over AI estimates. If responses feel generic or repetitive, you're likely in yellow/red zone regardless of calculated utilization.

---

**Up Next**: In Lesson 2, you'll learn to recognize degradation symptoms systematically, comparing healthy sessions to degraded sessions and building your diagnostic intuition.
