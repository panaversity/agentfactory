---
title: "Teach Claude Your Way of Working"
sidebar_position: 4
chapter: 5
lesson: 4
duration_minutes: 15

# PEDAGOGICAL LAYER METADATA
primary_layer: "Layer 1"
layer_progression: "L1 (Conceptual Foundation) → L2 (Recognition)"
layer_1_foundation: "Understanding skills as encoded expertise, distinguishing from prompts/CLAUDE.md/subagents, recognizing automatic activation"
layer_2_collaboration: "N/A (preparation for L2 in Lesson 06)"
layer_3_intelligence: "N/A"
layer_4_capstone: "N/A"

# HIDDEN SKILLS METADATA
skills:
  - name: "Understanding Skill-Based AI Customization"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can identify repeated procedures in their workflow, distinguish skills from alternative mechanisms, and articulate why encoding expertise matters"

learning_objectives:
  - objective: "Identify tasks in your workflow where you repeatedly explain the same preferences"
    proficiency_level: "A2"
    bloom_level: "Analyze"
    assessment_method: "List 3 personal tasks with repeated explanation patterns"
  - objective: "Distinguish skills from prompts, CLAUDE.md context, and subagents"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Explain when to use each mechanism"
  - objective: "Understand that skills encode reasoning patterns, not just commands"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Articulate the difference with a concrete example"
  - objective: "Recognize skill activation when Claude applies a skill automatically"
    proficiency_level: "A2"
    bloom_level: "Recognize"
    assessment_method: "Identify when Claude uses a skill vs. base knowledge"
  - objective: "Prepare for skill creation by mapping a personal procedure"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Document a procedure ready for encoding"

# Cognitive load tracking
cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (skills as expertise, reasoning patterns, cross-platform portability, automatic activation, comparison to alternatives, institutional capture) - within A2-B1 limit"

# Differentiation guidance
differentiation:
  extension_for_advanced: "Map multiple procedures across different domains; analyze which would benefit most from cross-platform portability"
  remedial_for_struggling: "Focus on one clear example (meeting notes); use the 'procedure vs. prompt' distinction as the primary mental model"

# Generation metadata
generated_by: "content-implementer v1.0.0 (043-lesson-04-skills-introduction)"
source_spec: "specs/043-lesson-04-skills-introduction/spec.md"
created: "2025-12-17"
last_modified: "2025-12-17"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"

# Legacy compatibility
prerequisites:
  - "Lessons 01-03: Claude Code installed and working"
  - "Basic experience with Claude Code interactions"
---

# Teach Claude Your Way of Working

You've been using Claude Code for a week. You notice something: you keep explaining the same things.

"When you write code, use TypeScript, not JavaScript. Add comments explaining WHY, not WHAT. Keep functions under 20 lines."

Or maybe it's not code at all: "When you summarize meeting notes, put action items first. Use bullet points, not paragraphs. Highlight decisions in bold."

You might think: "I should save this prompt somewhere and paste it each time."

That instinct is 10% of the answer—and missing 90% of the opportunity.

---

## The Repetition Tax

Every time you re-explain your preferences, you pay a tax. Not just in typing—in context.

Claude Code starts each session fresh. It doesn't remember that you prefer TypeScript over JavaScript, or that your meeting notes always have action items at the top. The conversation history helps within a session, but across sessions? You're starting over.

You've already learned about CLAUDE.md files (Lesson 07 covers them in depth). They help—persistent context that loads automatically. But CLAUDE.md is about *project context*: what this codebase does, what architecture it follows, what conventions matter here.

Your *procedures* are different. They're not project-specific. They're YOU-specific.

How you structure meeting notes. How you approach code review. How you write blog posts. How you organize research. These patterns follow you across projects, across tools, across years.

The real cost of repetition isn't typing. It's explaining HOW you think—not just WHAT you want—every single time.

What if you could explain once and have it apply forever?

---

## Prompts vs. Procedures

Here's a distinction that changes how you think about AI assistance.

**A prompt is a command:**

> "Write a blog post about sustainable living."

**A procedure is a reasoning pattern:**

> When writing blog posts:
> 1. Start with a counterintuitive hook that challenges assumptions
> 2. Structure as problem → failed solutions → insight → application
> 3. Use specific numbers over vague claims ("30% reduction" not "significant impact")
> 4. End with one actionable step, not three
> 5. Headlines: curiosity-driven, never clickbait

The prompt gets you *a* blog post. The procedure gets you *your* blog post—the one that sounds like you wrote it.

**The distinction matters because:**

Prompts encode WHAT you want. Procedures encode HOW you think about getting there.

The "how" includes preferences, constraints, quality bars, and decision logic. It's everything that would take 20 minutes to explain from scratch. It's the accumulated wisdom from doing this task hundreds of times.

**Consider meeting notes.** You've refined your approach over years:

- Action items at the top (owners, deadlines)
- Decisions highlighted (with who made them)
- Discussion points summarized (not transcribed verbatim)
- Open questions flagged for follow-up
- Maximum one page, regardless of meeting length

That procedure reflects hard-won knowledge about what makes notes useful. Claude doesn't have that knowledge—unless you teach it.

You could explain this procedure every session. Or you could explain it once, permanently.

---

## What Skills Actually Are

A skill is a folder of instructions that teaches Claude how to think about a specific type of task.

**Simple definition**: Encoded expertise. A skill captures how you approach a task—the reasoning pattern, the preferences, the quality criteria—so Claude can apply it automatically.

**What skills are NOT**: Saved prompts you paste in.

The difference is crucial. You don't invoke a skill by name (though you can). Skills are *discovered automatically*. You just work, and Claude recognizes when your encoded expertise applies.

"Help me write meeting notes from this transcript."

If you have a meeting-notes skill, Claude loads it. Your procedure activates. The output matches your standards—without you mentioning the skill at all.

**The expertise capture insight:**

Think about what you know that's hard to teach:
- A senior developer's code review instincts
- An editor's sense of when a sentence needs cutting
- A project manager's meeting note structure
- A researcher's citation evaluation criteria

These aren't commands you can type. They're judgment patterns built over years. Skills encode judgment.

When a new team member joins, they don't instantly have your instincts. They learn over months by watching, asking, making mistakes. Skills compress that learning curve—not for humans, but for your AI partnership.

But you already have tools for persistent context and complex tasks. When do skills specifically make sense?

---

## When Skills Beat the Alternatives

You have four mechanisms for customizing Claude's behavior. Each serves a different purpose.

| Mechanism | Best For | Key Limitation |
|-----------|----------|----------------|
| **One-off prompts** | Unique, exploratory tasks | No persistence; re-explain every time |
| **CLAUDE.md** | Project context, codebase patterns | Static; doesn't contain task procedures |
| **Subagents** | Complex, multi-step isolated tasks | Requires explicit invocation |
| **Skills** | Repeatable procedures, automatic application | Requires upfront definition |

**The decision framework:**

**Use a prompt** when the task is one-time or exploratory. You're figuring out what you want. You don't have a refined procedure yet.

**Use CLAUDE.md** when context persists across a project. Coding standards, architecture decisions, project-specific conventions. This is "what's true about THIS codebase."

**Use a subagent** when the task is complex, needs guaranteed execution, or requires isolated context. A comprehensive security audit. A multi-file refactoring. Something that deserves its own conversation space.

**Use a skill** when you have a procedure you've refined through experience and want applied consistently, automatically, across any relevant task. The key word is *automatically*—you don't have to remember to invoke it.

**Example: Code Review**

- **Prompt**: "Check this code for issues." (Generic, one-time)
- **CLAUDE.md**: Document your project's security standards. (Project-specific context)
- **Subagent**: "Run comprehensive security audit with isolated context." (Complex, guaranteed execution)
- **Skill**: Your review procedure—what to check, in what order, what severity levels—applied automatically when code review is mentioned. (Repeatable, automatic)

The skill activates when relevant. The subagent activates when invoked. CLAUDE.md provides background. The prompt is ephemeral.

Skills get even more powerful when you realize where they work.

---

## Write Once, Use Everywhere

The same skill runs across Claude.ai, Claude Code, and the API without modification.

This is the portability insight that makes skills worth the upfront investment.

You define a meeting-notes skill once. That skill works:
- In **Claude.ai** when you paste a transcript in the web interface
- In **Claude Code** when you're working in a project directory
- Through the **API** when you build automations or integrations

No modifications. No platform-specific versions. One definition, three surfaces.

**Why this matters:**

Your expertise doesn't live in one tool. It lives wherever you work. You might draft a blog post in Claude.ai, review code in Claude Code, and process data through the API. Your procedures should follow you.

Skills also stack. Claude can activate multiple skills in a single task. Writing a blog post about a technical topic? Your blog-writing skill AND your technical-explanation skill can both apply. Claude determines which capabilities are relevant—no manual selection required.

You might still be thinking: "This sounds like a developer thing. I don't write code."

Let's fix that assumption.

---

## Skills Beyond Code

Skills work for any repeated task—writing, research, accountability, learning—not just programming.

**The Accountability Buddy Example:**

A developer created a "streak" skill that functions as a personal accountability tracker. Not for code—for any goal:

- Reading 12 books a year
- Shipping side projects
- Meditation practice
- Workout consistency

The skill asks type-appropriate check-in questions. For reading goals: "What did you read? Key takeaway?" For fitness: "Did you complete today's workout? How do you feel?" For building: "What did you ship? What's blocking progress?"

One skill, multiple challenge types. The skill encodes the *accountability procedure*—how to check in, what to track, how to surface progress over time.

The developer noted: "Bite-sized daily progress adds up fast." Two weeks of using the skill yielded 10+ shipped projects through consistent accountability.

**Other non-coding examples:**

| Domain | What the Skill Encodes |
|--------|------------------------|
| **Writing** | Your editing procedure: what to cut, what to expand, voice consistency checks |
| **Research** | Citation standards, source evaluation criteria, note structure |
| **Communication** | Email templates, meeting note formats, status update structures |
| **Learning** | Note-taking method, flashcard creation rules, review scheduling |

**The pattern:**

If you have a procedure you've refined through experience—something that takes effort to explain to a new team member—it's a candidate for a skill.

The procedures don't need to be complex. "Meeting notes with action items first" is enough. "Blog posts that challenge assumptions before presenting solutions" is enough. The value comes from consistent application, not sophistication.

You now understand what skills are and why they matter. Before you learn *how* to create them (that's Lesson 06), let's prepare.

---

## Mapping Your First Procedure

This exercise prepares you for Lesson 06, where you'll write your first skill.

**Step 1: Identify Repetition**

Think about your last week with Claude (or any AI assistant):
- What did you explain more than once?
- What preferences did you keep restating?
- What quality criteria do you always add?

Write down three tasks where you repeated yourself.

**Step 2: Pick One and Articulate It**

Choose the task with the clearest, most stable procedure. Answer these questions:

1. **When does this task come up?** (What triggers it?)
2. **What steps do you follow?** (The procedure itself)
3. **What makes the output "yours" vs. generic?** (Your distinctive preferences)
4. **What would someone need to know to do it your way?** (The implicit knowledge)

Write your answers. Don't worry about formatting—you're capturing the procedure, not writing the skill yet.

**Step 3: Reality Check**

Ask yourself:
- Is this procedure stable, or still evolving rapidly?
- Would automatic activation help, or do you need explicit control?
- Is it distinct enough that Claude could recognize when it's relevant?

If yes to all three: you've found your first skill candidate.

**Example: Meeting Notes Procedure**

> **When**: After any meeting where I took notes or received a transcript
>
> **Steps**: (1) Extract action items with owners and deadlines, (2) Highlight decisions made, (3) Summarize discussion points without transcribing, (4) Flag open questions, (5) Keep to one page
>
> **Distinctive**: Action items FIRST (not buried), one page max (ruthless prioritization), owner names always included (accountability)
>
> **Implicit knowledge**: People skim notes. Action items are what matter. Nobody reads page 3.

That's a procedure ready for encoding.

---

## What's Ahead

Lesson 06 shows you how to turn procedures into working skills:

- The SKILL.md format and YAML metadata
- The three-level loading architecture (how Claude discovers skills efficiently)
- Hands-on: Create a blog-planner skill from scratch
- Skills vs. subagents: detailed decision criteria
- Co-learning refinement: improving skills through iteration with Claude

You'll go from understanding skills conceptually to having a working skill in your `.claude/skills/` folder.

The procedure you mapped in this lesson? That's your raw material for Lesson 06.

---

## Try With AI

**Identify Your Skill Candidates:**

> "I've been using Claude for [describe your work: writing, coding, research, project management, etc.]. Help me identify 3 procedures I repeat that would make good skills. For each one, explain: what the skill would do, when it would activate automatically, and what makes my approach distinctive from generic output."

**Understand the Activation Mechanism:**

> "Explain how Claude decides when to activate a skill automatically. I want to understand the difference between: (1) Claude using a skill I've defined, (2) Claude using its base knowledge, and (3) Claude following a prompt I gave this session. What signals trigger skill activation vs. just responding normally?"

**Prepare Your First Skill:**

> "I want to create a skill for [your task: meeting notes, code review, blog writing, etc.]. Before I write any skill files, help me articulate my procedure. Ask me questions about: my steps, my preferences, what makes my output distinctive, and what implicit knowledge I have. Then document this as a 'procedure specification' I can use when building the actual skill."

**Evaluate Skill vs. Alternatives:**

> "I have this repeated task: [describe task]. Help me decide: should this be a skill (automatic activation), a subagent (explicit invocation, isolated context), or just a prompt I save? Walk through the decision criteria and recommend an approach with reasoning."

---

## Reflection

The shift from "ask Claude each time" to "teach Claude once" mirrors how expertise scales in organizations.

A senior team member doesn't re-explain their approach to every new hire individually. They document it, encode it in processes, make it transferable. The knowledge stops living in one person's head and becomes organizational capability.

Skills do the same for your AI partnership. Your procedures—refined over years—become persistent, portable, automatic.

The question isn't whether you have procedures worth encoding. You do. Everyone who's good at their work has accumulated judgment patterns.

The question is which one you'll encode first.
