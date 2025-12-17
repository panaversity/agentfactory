---
title: "Teach Claude Your Way of Working"
sidebar_position: 4
chapter: 5
lesson: 4
duration_minutes: 20

# PEDAGOGICAL LAYER METADATA
primary_layer: "Layer 1"
layer_progression: "L1 (Conceptual Foundation) → L2 (Hands-On Experience)"
layer_1_foundation: "Understanding skills as encoded expertise, recognizing automatic activation, seeing real-world applications"
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
    measurable_at_this_level: "Student can identify repeated procedures in their workflow, understand skill benefits, and experience automatic activation"

learning_objectives:
  - objective: "Understand that skills encode reasoning patterns, not just commands"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Articulate the difference with a concrete example"
  - objective: "Experience skills in action through hands-on practice"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Successfully use skills from the Skills Lab"
  - objective: "Identify tasks in your workflow where you repeatedly explain the same preferences"
    proficiency_level: "A2"
    bloom_level: "Analyze"
    assessment_method: "List 3 personal tasks with repeated explanation patterns"

# Cognitive load tracking
cognitive_load:
  new_concepts: 4
  assessment: "4 concepts (skills as expertise, automatic activation, procedure mapping, skill creation prep) - well within A2-B1 limit"

# Differentiation guidance
differentiation:
  extension_for_advanced: "Map multiple procedures across different domains; experiment with all 8 skills in the lab"
  remedial_for_struggling: "Focus on the Dr. Claude analogy and hands-on experience first"

# Generation metadata
generated_by: "content-implementer v1.0.0 (043-lesson-04-skills-introduction)"
source_spec: "specs/043-lesson-04-skills-introduction/spec.md"
created: "2025-12-17"
last_modified: "2025-12-17"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "3.0.0"

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

Claude Code starts each session fresh. It doesn't remember that you prefer TypeScript over JavaScript, or that your meeting notes always have action items at the top. Within a session, conversation history helps. Across sessions? You're starting over.

The real cost isn't typing. It's explaining HOW you think—not just WHAT you want—every single time.

What if you could explain once and have it apply forever?

---

## What Skills Actually Are

Think about the difference between a general practitioner and a cardiologist.

Both are doctors. Both went to medical school. But when you walk in with chest pain, the cardiologist doesn't start from "what is a heart?" They have years of specialized pattern recognition—which symptoms cluster together, which tests to order first, which treatments work for which patient profiles.

That specialized knowledge is the difference between competent and expert.

**Claude without skills**: A brilliant general practitioner. Knows a lot about everything. Can help with anything. But approaches every task from first principles.

**Claude with skills**: A specialist. When you mention meeting notes, Claude doesn't think "what are meeting notes?" It thinks "action items first, one page max, owners and deadlines for every item"—because that's YOUR procedure, loaded automatically.

**Simple definition**: A skill is encoded expertise. It captures how YOU approach a specific task—the reasoning pattern, the preferences, the quality criteria—so Claude applies it automatically.

**What skills are NOT**: Saved prompts you paste in.

The difference is crucial. You don't invoke a skill by name (though you can). Skills are *discovered automatically*. You just work, and Claude recognizes when your encoded expertise applies.

"Help me write meeting notes from this transcript."

If you have a meeting-notes skill, Claude loads it. Your procedure activates. The output matches your standards—without you mentioning the skill at all.

---

## Hands-On: Experience Skills in Action

Enough theory. Let's feel what skills do.

We've prepared a Skills Lab with 8 ready-to-use skills. Download it and experience automatic skill activation firsthand.

### Step 1: Download the Skills Lab

1. Go to [github.com/panaversity/claude-code-skills-lab](https://github.com/panaversity/claude-code-skills-lab)
2. Click the green **Code** button
3. Select **Download ZIP**
4. Extract the ZIP file
5. Open the extracted folder in your terminal

### Step 2: Try a Skill

Open Claude Code in the skills lab directory:

```bash
claude
```

Then try this prompt:

> "Create a professional project proposal document for a mobile app redesign project. Include executive summary, timeline, and budget sections."

### Step 3: Notice What Happened

You didn't say "use the docx skill." Claude recognized the task and activated the skill automatically.

Try another:

> "Help me write a company newsletter announcing our new AI development initiative. Keep it under 500 words."

Again—automatic activation. The `internal-comms` skill loaded because the task matched.

**Ask Claude directly:**
> "Which skills did you use in our conversation so far? How did you decide when to activate each one?"

This reveals the skill discovery mechanism in action.

### Available Skills in the Lab

| Skill | What It Does |
|-------|--------------|
| `docx` | Create and edit Word documents with formatting |
| `pptx` | Build PowerPoint presentations with layouts |
| `xlsx` | Create spreadsheets with formulas and analysis |
| `pdf` | Extract text, merge PDFs, handle forms |
| `internal-comms` | Templates for newsletters, reports, FAQs |
| `skill-creator` | Guide for building your own skills |

---

## Real Example: The Accountability Buddy

Skills work for any repeated task—not just documents or code.

A developer had a problem: out of the 1001 personal learning topics, build ideas, and reading lists they'd been wanting to tackle—what had they actually achieved? They'd lost track. Time kept slipping by.

Their first thought: "Not another task tracker. Not the 999th app I've tried and given up on within a week."

So they built a Claude Code skill instead. No app. No new tool. Just a skill that turns Claude into an accountability partner.

**The insight:** The file structure stays universal—only the content adapts.

| Challenge Type | What the Skill Asks |
|----------------|---------------------|
| **Learning** | "Any aha moments? Progress on milestones?" |
| **Building** | "What did you ship? Any blockers?" |
| **Fitness** | "What exercises? How did your body feel?" |
| **Habit** | "Did you complete it? How did it feel?" |

Asking "what did you ship?" to someone tracking meditation is useless. Asking "how did it feel?" makes sense. The skill captures this distinction upfront.

**Results after two weeks:** 10+ micro-apps shipped. Topics that had been sitting on their "someday" list for months finally got attention.

"Bite-sized daily progress adds up fast."

---

## Mapping Your First Procedure

This exercise prepares you for Lesson 06, where you'll create your first skill.

**Step 1: Identify Repetition**

Think about your last week with Claude:
- What did you explain more than once?
- What preferences did you keep restating?
- What quality criteria do you always add?

Write down three tasks where you repeated yourself.

**Step 2: Pick One and Articulate It**

Choose the task with the clearest procedure. Answer:

1. **When does this task come up?**
2. **What steps do you follow?**
3. **What makes the output "yours" vs. generic?**
4. **What would someone need to know to do it your way?**

**Example: Meeting Notes**

> **When**: After any meeting with notes or a transcript
>
> **Steps**: (1) Extract action items with owners and deadlines, (2) Highlight decisions, (3) Summarize discussion points, (4) Flag open questions, (5) Keep to one page
>
> **Distinctive**: Action items FIRST (not buried), one page max, owner names always included
>
> **Implicit knowledge**: People skim notes. Action items are what matter. Nobody reads page 3.

That's a procedure ready for encoding.

---

## Try With AI

**Identify Your Skill Candidates:**

> "I've been using Claude for [describe your work]. Help me identify 3 procedures I repeat that would make good skills. For each one, explain: what the skill would do, when it would activate automatically, and what makes my approach distinctive."

**Prepare Your First Skill:**

> "I want to create a skill for [your task: meeting notes, code review, blog writing, etc.]. Before I write any skill files, help me articulate my procedure. Ask me questions about my steps, preferences, and implicit knowledge. Then document this as a 'procedure specification' I can use when building the actual skill."

---

## Reflection

The shift from "ask Claude each time" to "teach Claude once" mirrors how expertise scales in organizations.

A senior team member doesn't re-explain their approach to every new hire individually. They document it, encode it in processes, make it transferable. The knowledge stops living in one person's head and becomes organizational capability.

Skills do the same for your AI partnership. Your procedures—refined over years—become persistent, portable, automatic.

The question isn't whether you have procedures worth encoding. You do.

The question is which one you'll encode first.
