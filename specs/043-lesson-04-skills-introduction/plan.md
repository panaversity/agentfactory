# Implementation Plan: Lesson 04 — Teach Claude Your Way of Working

**Feature ID**: 043-lesson-04-skills-introduction
**Version**: 1.0.0
**Created**: 2025-12-17

---

## Detailed Lesson Outline

### Frontmatter & Metadata
- Standard YAML with lesson 4 positioning
- Layer 1 primary, transitioning to Layer 2
- 6 concepts, A2-B1 proficiency
- Prerequisites: Lessons 01-03 (Claude Code installed and working)

---

## Section 1: The Repetition Tax

**Core Idea**: Every time you re-explain your preferences, you pay a tax—in time, context, and cognitive load.

**Opening Hook** (Challenge the Belief):
> You've been using Claude Code for a week. You notice something: you keep explaining the same things.
>
> "When you write code, use TypeScript, not JavaScript. Add comments explaining WHY, not WHAT. Keep functions under 20 lines."
>
> Or maybe it's not code: "When you summarize meeting notes, put action items first. Use bullet points, not paragraphs. Highlight decisions in bold."
>
> You might think: "I should save this prompt somewhere and paste it each time."
>
> That instinct is 10% of the answer—and missing 90% of the opportunity.

**Content**:
- The repetition problem isn't just typing—it's context
- Each new session, Claude starts fresh
- CLAUDE.md helps (persistent context), but has limits
- The real cost: explaining HOW you think, not just WHAT you want

**Transition** (Open Curiosity):
The difference between "what you want" and "how you think" is everything. Let's examine that gap.

---

## Section 2: Prompts vs. Procedures

**Core Idea**: Prompts say "do this." Procedures say "here's how to think about this type of problem."

**Content**:

**A prompt is a command**:
```
Write a blog post about sustainable living.
```

**A procedure is a reasoning pattern**:
```
When writing blog posts:
1. Start with a counterintuitive hook that challenges assumptions
2. Structure in problem → failed solutions → insight → application
3. Use specific numbers over vague claims ("30% reduction" not "significant impact")
4. End with one actionable step, not three
5. Headlines: curiosity-driven, never clickbait
```

The prompt gets you *a* blog post. The procedure gets you *your* blog post—the one that sounds like you wrote it.

**Key Insight**:
> Prompts encode WHAT. Procedures encode HOW.
>
> The "how" includes preferences, constraints, quality bars, and decision logic that would take 20 minutes to explain from scratch.

**Concrete Example** (Non-Coding):

You take meeting notes. Your procedure:
- Action items at the top (owners, deadlines)
- Decisions highlighted (with who made them)
- Discussion points summarized (not transcribed)
- Open questions flagged for follow-up
- Maximum one page

That procedure reflects years of iteration. You know what makes notes useful. Claude doesn't—unless you teach it.

**Transition**:
So you could explain this procedure every session. Or you could explain it once, permanently. That's what skills do.

---

## Section 3: What Skills Actually Are

**Core Idea**: A skill is a folder of instructions that teaches Claude how to think about a specific type of task.

**Content**:

**Simple Definition**:
> A skill is encoded expertise. It captures how you approach a task—the reasoning pattern, the preferences, the quality criteria—so Claude can apply it automatically.

**Not a Prompt Library**:
Skills aren't "saved prompts you paste in." They're discovered automatically. You don't invoke a skill by name. You just work, and Claude recognizes when your encoded expertise applies.

"Help me write meeting notes from this transcript."

If you have a meeting-notes skill, Claude loads it automatically. Your procedure activates. The output matches your standards.

**The Expertise Capture Insight**:

Think about what you know that's hard to teach:
- A senior developer's code review instincts
- An editor's sense of when a sentence needs cutting
- A project manager's meeting note structure
- A researcher's citation standards

These aren't commands. They're judgment patterns. Skills encode judgment.

**Transition**:
But wait—you already have CLAUDE.md for context and subagents for complex tasks. When do skills make sense?

---

## Section 4: When Skills Beat the Alternatives

**Core Idea**: Skills fill a specific gap that prompts, CLAUDE.md, and subagents don't cover.

**Content**:

| Mechanism | Best For | Limitation |
|-----------|----------|------------|
| **One-off prompts** | Unique, exploratory tasks | No persistence; re-explain every time |
| **CLAUDE.md** | Project context, codebase patterns | Static; doesn't contain task-specific procedures |
| **Subagents** | Complex, multi-step isolated tasks | Explicit invocation; isolated context |
| **Skills** | Repeatable procedures you want applied automatically | Requires upfront definition |

**Decision Framework**:

**Use a prompt** when the task is one-time or exploratory.

**Use CLAUDE.md** when context persists across a project (coding standards, architecture).

**Use a subagent** when the task is complex, needs guaranteed execution, or requires isolated context.

**Use a skill** when you have a procedure you've refined over time and want applied consistently, automatically, across any relevant task.

**Example Decision**:

> "I want Claude to review code securely."
>
> - **Prompt**: "Check this code for security issues." (Generic, one-time)
> - **CLAUDE.md**: Document your security standards for this project (Project-specific)
> - **Subagent**: "Run comprehensive security audit" (Complex, guaranteed execution)
> - **Skill**: Your security review procedure—what to check, in what order, what severity levels—applied automatically whenever code review is mentioned (Repeatable, automatic)

**Transition**:
Skills get even more powerful when you realize they work everywhere—not just in Claude Code.

---

## Section 5: Write Once, Use Everywhere

**Core Idea**: The same skill runs across Claude.ai, Claude Code, and the API without modification.

**Content**:

This is the portability insight that makes skills worth the upfront investment.

You define a meeting-notes skill once. That skill works:
- In Claude.ai when you paste a transcript
- In Claude Code when you're in a project directory
- Through the API when you build automations

No modifications. No platform-specific versions. One definition, three surfaces.

**Why This Matters**:

Your expertise doesn't live in one tool. It lives wherever you work. Skills make your encoded procedures portable.

**Stack Multiple Skills**:

Claude can activate multiple skills in a single task. Writing a blog post about a technical topic? Your blog-writing skill AND your technical-explanation skill can both apply. Claude figures out which capabilities are relevant—no manual selection.

**Transition**:
You might still be thinking: "This sounds like a developer thing. I don't write code." Let's fix that assumption.

---

## Section 6: Skills Beyond Code

**Core Idea**: Skills work for any repeated task—writing, research, accountability, learning—not just coding.

**Content**:

**The Accountability Buddy Example**:

A developer created a "streak" skill that functions as a personal accountability tracker. Not for code—for any goal:
- Reading 12 books a year
- Shipping side projects
- Meditation practice
- Workout consistency

The skill asks type-appropriate check-in questions:
- For reading: "What did you read? Key takeaway?"
- For fitness: "Did you complete today's workout? How do you feel?"
- For building: "What did you ship? What's blocking progress?"

One skill, multiple challenge types. The skill encodes the *accountability procedure*—how to check in, what to track, how to surface progress.

**Other Non-Coding Examples**:

| Domain | Skill Example |
|--------|---------------|
| **Writing** | Your editing procedure: what to cut, what to expand, voice consistency |
| **Research** | Citation standards, source evaluation criteria, note structure |
| **Communication** | Email templates, meeting note formats, status update structures |
| **Learning** | Note-taking method, flashcard creation rules, review scheduling |

**Key Insight**:
> If you have a procedure you've refined through experience—something that takes effort to explain to a new team member—it's a candidate for a skill.

**Transition**:
You now understand what skills are and why they matter. Before we get into *how* to create them (that's the next lesson), let's prepare.

---

## Section 7: Mapping Your First Procedure

**Core Idea**: Preparation exercise—identify the procedure you'll encode as your first skill.

**Content**:

This is homework that pays off in Lesson 06.

**Step 1: Identify Repetition**

Think about your last week with Claude (or any AI assistant):
- What did you explain more than once?
- What preferences did you keep restating?
- What quality criteria do you always add?

**Step 2: Articulate the Procedure**

Pick one repeated task. Write down:
1. When does this task come up?
2. What steps do you follow?
3. What makes the output "yours" vs. generic?
4. What would someone need to know to do it your way?

**Step 3: Reality Check**

Ask yourself:
- Is this procedure stable, or still evolving?
- Would you benefit from automatic activation?
- Is it distinct enough to be recognizable?

If yes to all three: you've found your first skill candidate.

**Try With AI**:

> "I want to identify procedures I should encode as skills. Help me analyze my workflow. Ask me about: (1) What tasks do I repeat weekly? (2) What preferences do I always specify? (3) What would a new team member need to learn to work like me? Based on my answers, suggest 3 skill candidates ranked by how much time they'd save."

**Transition** (to Lesson 06):
You've mapped a procedure. You understand why skills matter. Now you're ready to build one. Lesson 06 shows you the SKILL.md format, the three-level architecture, and walks you through creating your first working skill.

---

## Section 8: What's Ahead

**Brief Preview** (No Summarizing):

Lesson 06: Agent Skills
- SKILL.md format and YAML frontmatter
- The three-level loading architecture
- Hands-on: Create a blog-planner skill
- Skills vs. subagents: when to use which
- Co-learning refinement with Claude

You'll go from understanding skills conceptually to having a working skill in your `.claude/skills/` folder.

---

## Try With AI Section

**Identify Your Skill Candidates**:
> "I've been using Claude for [describe your work]. Help me identify 3 procedures I repeat that would make good skills. For each one, describe: what the skill would do, when it would activate, and what makes my way distinctive from generic output."

**Understand the Mechanism**:
> "Explain how Claude decides when to activate a skill automatically. I want to understand the difference between: (1) Claude using a skill, (2) Claude using its base knowledge, and (3) Claude following a prompt I gave this session. What signals trigger skill activation?"

**Prepare Your First Skill**:
> "I want to create a skill for [your task]. Before I write any SKILL.md file, help me articulate my procedure: What are my steps? What are my preferences? What makes my output distinctive? Document this as a 'procedure specification' I can reference when I build the actual skill."

---

## Reflection Prompt (Not a Summary)

The shift from "ask Claude each time" to "teach Claude once" mirrors how expertise scales in organizations.

A senior team member doesn't re-explain their approach to every new hire individually. They document it, encode it in processes, make it transferable.

Skills do the same for your AI partnership. The question isn't whether you have procedures worth encoding—you do. The question is which one you'll encode first.

---

## Implementation Notes

### Files to Create
- `04-teach-claude-your-way.md` — Main lesson content
- `04-teach-claude-your-way.summary.md` — Key concepts for revision

### Validation Checks
- [ ] No SKILL.md syntax in lesson
- [ ] No three-level architecture details
- [ ] At least 2 non-coding examples (accountability, meeting notes)
- [ ] Belief challenged before correct framing (Section 1)
- [ ] No summarizing phrases
- [ ] Each section ends by opening curiosity
- [ ] Emotional arc: skeptical → curious → confident

### Cross-References
- Lesson 03: Free Claude Setup (prerequisite)
- Lesson 06: Agent Skills (continuation—HOW to create)
- Lesson 07: CLAUDE.md (comparison mechanism)
- Lesson 09: Subagents (comparison mechanism)
