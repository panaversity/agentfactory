---
title: "Lesson 2: Understanding Context Windows"
chapter: 10
lesson: 2
duration_minutes: 18
sidebar_position: 2
description: "Learn about context window limitations and recognize the signs of context degradation"
keywords: [context window, context rot, AI memory, token limits, performance degradation]

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Understanding Context Window Constraints"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain what a context window is, why it's finite, and compare sizes across different AI tools"

  - name: "Recognizing Context Degradation"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify symptoms of context degradation (inconsistent code, repetition, errors) and recognize when context window is filling up"

  - name: "Managing AI Memory Limitations"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can describe what happens when context windows fill (performance degradation, lost information) and understand why AI 'forgets'"

learning_objectives:
  - objective: "Explain what a context window is and why it's finite"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Student description using analogies (desk space, working memory) and comparison of context window sizes"

  - objective: "Describe what happens when context windows fill up"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Explanation of degradation symptoms (performance drop, inconsistency, information loss)"

  - objective: "Recognize the signs of context degradation in your AI sessions"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Identification of degradation patterns in real AI conversations (5+ symptoms)"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (Context Window definition, Token measurement, Context Window sizes, Context degradation symptoms, Context rot) within A1-A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Research how context windows are implemented technically (attention mechanisms, transformer architecture); compare RAG (Retrieval-Augmented Generation) vs raw context approaches"
  remedial_for_struggling: "Focus on desk analogy throughout; use only Claude Code and Gemini CLI examples (skip ChatGPT to reduce cognitive load); practice with one symptom at a time"
---

# Understanding Context Windows

## Your AI's Working Memory

Remember from Lesson 1 that context is like the information environment your AI operates in. Now let's understand the **constraints** of that environment.

### The Physical Memory Analogy

Think about your own working memory when studying:

**Scenario:** You're studying for an exam with notes spread across your desk.

**Early in Study Session:**
- Your desk has space
- You can see all your materials easily
- You remember everything you just read
- You can quickly reference earlier topics

**As Your Desk Fills Up:**
- Notes start piling on top of each other
- You can't see older materials as easily
- You forget details from earlier
- You slow down as you search through papers
- Some papers fall off the desk completely!

**Your AI's context window works exactly the same way.** It has limited "desk space" for holding information, and as it fills up, performance degrades.

---

## What is a Context Window?

### WHAT: The Technical Definition

A **context window** is the maximum amount of information your AI coding agent can "remember" and work with at one time.

**Key Facts:**

| AI Tool | Context Window Size | Rough Equivalent |
|---------|---------------------|------------------|
| **Claude Sonnet 4.5** | ~200,000 tokens | ~150,000 words or ~500 book pages |
| **Gemini 1.5 Pro** | 1,000,000 tokens | ~750,000 words or ~2,500 book pages |
| **ChatGPT (GPT-4)** | ~16,000 tokens* | ~12,000 words or ~40 book pages |

**What's a "token"?** For now, think of it as roughly 3/4 of a word. So 1,000 tokens ≈ 750 words.

\* *Note: Context window sizes as of November 2025. These increase over time—check current specs when using tools.*

### What Fills the Context Window?

When you work with an AI coding agent, these things take up space in the context window:

1. **Your conversation history**
   - Every prompt you've sent
   - Every response the AI gave
   - All the back-and-forth discussion

2. **Files the AI has read**
   - Code files you asked it to analyze
   - Documentation it looked at
   - Configuration files it examined

3. **Tool outputs**
   - Results from terminal commands
   - File listings
   - Test results

4. **System instructions**
   - Built-in rules for how the AI behaves
   - Any custom instructions you set

**Important Insight:**
Everything in the context window is "visible" to the AI.  
Anything outside the context window **doesn't exist** to the AI.

---

## Why Context Windows Matter

### The Three Key Characteristics

Understanding these three facts will change how you work with AI:

#### 1. Everything in Context is Visible

If you loaded a file 20 messages ago, the AI can still "see" it—assuming it's still within the context window.

**Example:**
```bash
# Message 1: Load context
"Read src/auth/oauth.py to understand our authentication pattern"

# Message 15: Reference it
"Create a new auth function following the OAuth pattern you saw earlier"

# AI can reference it because it's still in context!
```

#### 2. Anything Outside Context Doesn't Exist

If something isn't in the context window, the AI literally cannot access it.

**Example:**
```bash
# New session (fresh context window)
"Continue the authentication function we were building yesterday"

# AI Response: "I don't have context about a previous authentication 
# function. Could you provide more details or share the code?"
```

**Why?** Each session starts with an empty context window. Yesterday's conversation isn't there unless you reload it.

#### 3. Context Degrades as It Fills

As the context window fills up, the AI's performance gets worse. This is called **"context rot"**.

---

## Context Rot: The Performance Problem

This is the most important concept in this lesson.

### WHAT: Context Rot Explained

**Context rot** is the gradual degradation of AI performance as the context window fills up.

**The Pattern:**

```
Session Start (Empty Context) 
        ↓
    [20% Full]
    ✓ Perfect performance
    ✓ AI remembers everything
    ✓ Fast, accurate responses
        ↓
    [60% Full]
    ⚠ Starting to struggle
    ⚠ May miss some details
    ⚠ Slower processing
        ↓
    [90% Full]
    ❌ Significant problems
    ❌ Forgets earlier decisions
    ❌ May contradict itself
    ❌ Much slower responses
```

### WHY: Why Does This Happen?

Let's understand the technical reason in simple terms:

**The Attention Mechanism:**

When AI processes text, it uses something called a "transformer attention mechanism." Here's what that means in plain English:

**Non-Programmer Analogy:**
Imagine you're in a classroom with 10 students. The teacher can pay attention to everyone and remember who said what. Easy!

Now imagine 100 students. The teacher struggles to track everything. Possible, but harder.

Now imagine 1,000 students. The teacher can't possibly remember everything or give everyone equal attention.

**Your AI works the same way:**
- **Fewer tokens in context** = Easy to "pay attention" to everything
- **More tokens in context** = Harder to track it all
- **Context nearly full** = Performance breaks down

**Technical Note:** The attention mechanism has to process every token against every other token. 10 tokens = 100 calculations. 1,000 tokens = 1,000,000 calculations. This is why more context = slower performance.

---

## Recognizing Context Rot

As a beginner, you don't need to count tokens. You just need to recognize the **warning signs** that context is degrading.

### The Warning Signs

**🟢 Healthy Context (Early Session):**
- ✓ AI responds quickly
- ✓ References earlier decisions accurately
- ✓ Consistent code style throughout
- ✓ No contradictions
- ✓ Remembers files you loaded

**🟡 Context Degrading (Mid Session):**
- ⚠ Slightly slower responses
- ⚠ Occasionally asks about things you already discussed
- ⚠ Minor inconsistencies starting to appear
- ⚠ May need reminders about earlier decisions

**🔴 Context Rot (Late Session):**
- ❌ Very slow responses
- ❌ Forgets decisions made 10-15 messages ago
- ❌ Contradicts previous statements
- ❌ Asks you to repeat information you already provided
- ❌ Generated code doesn't match earlier patterns
- ❌ Seems "confused" about project context

### Simple Test: The Recall Check

Want to check if context is degrading? Try this:

```bash
# Ask the AI to recall something from earlier
"What authentication pattern did we decide to use?"

# OR
"What were the three main components we built so far?"
```

**Healthy Context Response:**
"We're using OAuth 2.0 with JWT tokens, following the pattern in src/auth/oauth.py. The three main components we built are: 1) User authentication service, 2) Token management, 3) Permission checking middleware."

**Degraded Context Response:**
"Could you remind me what authentication pattern we discussed? I want to make sure I give you accurate information."

If the AI can't recall recent decisions, **context rot has started.**

---

## When to Worry About Context

### General Guidelines

For beginners working on typical projects:

**Session Duration:**
- **First hour:** Usually fine, don't worry
- **1-2 hours:** Start monitoring for signs
- **2+ hours:** Very likely experiencing some degradation

**Message Count:**
- **First 10-15 messages:** Healthy context
- **15-25 messages:** Watch for warning signs
- **25+ messages:** Context rot likely started

**File Count Loaded:**
- **1-5 files:** No problem
- **5-15 files:** Fine for most projects
- **15+ files:** Consider if you need them all
- **50+ files:** Almost certainly too much at once

### Different Tools, Different Limits

Remember, context window sizes vary:

**ChatGPT (Smallest Window - ~16K tokens):**
- Gets full fastest
- Need to manage context most carefully
- Great for learning and short tasks
- Plan to reload context more often

**Claude Code (Medium Window - ~200K tokens):**
- Can handle longer sessions
- Good for full features
- Still needs management for multi-hour sessions

**Gemini (Largest Window - ~1M tokens):**
- Can hold huge amounts of context
- Good for analyzing entire large codebases
- Still experiences degradation, just later

---

## Practical Example: Watching Context Fill

Let's walk through a real session and track context usage:

### Session Start (Context: Empty)

```bash
Message 1: "I'm building a Python FastAPI project. Read README.md"
Context Used: ~2,000 tokens (README file)
```

### Early Session (Context: 10% Full)

```bash
Messages 2-5: Discussion about project structure
Context Used: ~20,000 tokens (conversation history + README)
Performance: ✓ Excellent
```

### Mid Session (Context: 50% Full)

```bash
Messages 6-15: Loaded multiple files, built features
Context Used: ~100,000 tokens (history + 10 files + discussion)
Performance: ✓ Good, slight slowdown
Warning Sign: AI takes 2-3 seconds longer to respond
```

### Late Session (Context: 85% Full)

```bash
Messages 16-25: More features, more discussion
Context Used: ~170,000 tokens (history + files + code generated)
Performance: ⚠ Degrading
Warning Signs:
- AI asks: "Could you remind me which database we're using?"
- AI generates code not matching earlier style
- Takes 5-10 seconds to respond
```

### Critical (Context: 95% Full)

```bash
Message 26+: Continuing work
Context Used: ~190,000 tokens
Performance: ❌ Poor
Warning Signs:
- AI contradicts decision made 15 messages ago
- Forgets files you loaded earlier
- Very slow responses
- Code quality drops
```

**Action Needed:** Time to compress context or start fresh! (You'll learn how in Lesson 4)

---

## Pause and Reflect

Take a moment to think:

**Question 1:** In your own words, what is a context window?

**Question 2:** Can you explain context rot to someone else using the classroom analogy?

**Question 3:** If you're working with an AI and it starts asking you to repeat information you already provided, what might be happening?

---

## Try With AI

**Tool:** ChatGPT Web

Let's practice recognizing context concepts.

### Prompt 1: Understanding the Concept
Copy this into ChatGPT:

```
If I'm working with an AI coding agent and after an hour it starts 
giving me inconsistent answers and seems to forget things I told it 
earlier, what's probably happening? 

Explain using a simple analogy a non-programmer would understand.
```

**What to expect:**
- ChatGPT should describe context window filling up
- You'll get an analogy (like running out of desk space or RAM)
- Explanation of why performance degrades

**Check:** Can you identify the warning signs of context rot now?

---

### Prompt 2: Recognizing the Signs
Copy this into ChatGPT:

```
I'm working with Claude Code on a project. Here are some things 
I'm noticing:

1. I asked it to use FastAPI, and it keeps suggesting Flask
2. It's taking longer and longer to respond
3. It just asked me what database I'm using, but I told it 20 messages ago
4. The code it's generating doesn't match the style it was using earlier

What's happening? Is this context rot?
```

**What to expect:**
- ChatGPT should confirm these are classic context rot symptoms
- Explanation that context window is full or nearly full
- Suggestions that you might need to refresh context

**Reflection:** Which of these signs seem most important to watch for?

---

### Prompt 3: Planning Ahead
Copy this into ChatGPT:

```
I'm about to start a coding session with an AI assistant. I want to 
load context about my project. 

I have:
- README.md (2 pages)
- 30 Python files in src/
- 20 test files
- API documentation (50 pages)
- CONTRIBUTING.md (5 pages)

Should I load all of these at the start? Why or why not?
```

**What to expect:**
- ChatGPT should advise against loading everything at once
- Recommendation to load progressively as needed
- Explanation that too much context too fast causes problems


