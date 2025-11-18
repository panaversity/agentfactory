---
title: "Agent Skills"
sidebar_position: 6
chapter: 5
lesson: 6
duration_minutes: 9

# PEDAGOGICAL LAYER METADATA
primary_layer: "Layer 2"
layer_progression: "L2 (AI Collaboration)"
layer_1_foundation: "N/A"
layer_2_collaboration: "Co-learning skill refinement (Step 4), AI as Teacher suggesting improvements, Student as Teacher specifying constraints, convergence toward optimized skill design"
layer_3_intelligence: "N/A"
layer_4_capstone: "N/A"

# HIDDEN SKILLS METADATA
skills:
  - name: "Designing Reusable Agent Skills"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can create SKILL.md files with YAML frontmatter and clear instructions, write descriptions that trigger autonomous discovery, and refine skills through co-learning iteration"

learning_objectives:
  - objective: "Understand skills as reusable capabilities that extend Claude's knowledge"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of skill architecture and autonomous discovery mechanism"
  - objective: "Create a working SKILL.md file with YAML frontmatter and instructions"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Creation of functional skill file with proper structure and activation triggers"
  - objective: "Write effective skill descriptions that trigger autonomous discovery"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Skill description that successfully activates in appropriate contexts"
  - objective: "Improve skills through co-learning iteration with Claude"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Refinement of skill through AI collaboration demonstrating Three Roles convergence"
  - objective: "Recognize when to use skills vs subagents"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Decision analysis comparing skill and subagent appropriateness for given scenarios"

# Cognitive load tracking
cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (skills definition, three-level loading, autonomous discovery, SKILL.md structure, skills vs subagents, intelligence accumulation, co-learning refinement) - within B1 limit of 10 ✓"

# Differentiation guidance
differentiation:
  extension_for_advanced: "Create skill suites with interdependencies; design skills that compose with MCP servers for advanced workflows"
  remedial_for_struggling: "Start with blog-planner example from lesson; copy and adapt working skill structure before creating from scratch"

# Generation metadata
generated_by: "content-implementer v1.0.0 (029-chapter-5-refinement)"
source_spec: "specs/029-chapter-5-refinement/spec.md"
created: "2025-01-17"
last_modified: "2025-01-17"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"

# Legacy compatibility
prerequisites:
  - "Lessons 2-5: Claude Code, CLAUDE.md, MCP, Subagents"
  - "Understanding of reusable components"
---

# Agent Skills: Teaching Claude New Capabilities

You're working with Claude Code on multiple blog posts this week. Every time you start a new session, you give the same instructions:

"Create an outline with 5 main sections, suggest 5 headline variations, make the introduction hook readers in the first sentence, keep paragraphs under 4 sentences..."

By the third blog post, you're frustrated. **Why can't Claude just remember how you like blog posts structured?**

You could use a subagent—but that requires explicitly saying "Use the blog-planner subagent" every time. What if Claude could **automatically** apply your blog-planning workflow whenever you mention writing a blog post?

**That's what Skills solve.**

---

## What Are Agent Skills?

**Definition**: An Agent Skill is a **reusable capability** that extends Claude Code's knowledge in specific domains. When Claude recognizes the context (like "write a blog post"), it automatically loads and applies the skill's instructions.

**Think of skills as**: Teaching materials that Claude can reference when needed—like giving Claude a handbook on "how I want blog posts structured."

**Key characteristic**: Skills are **discovered autonomously**. You create the skill once (SKILL.md file), and Claude applies it automatically when relevant—no explicit invocation needed.

### How Skills Differ from Subagents

| Aspect | Subagents | Skills |
|--------|-----------|--------|
| **Context** | Separate (isolated conversation) | Shared (main conversation) |
| **Invocation** | Hard ("Use X subagent" guaranteed) | Soft (Claude decides when relevant) |
| **Best For** | Complex, isolated tasks | Lightweight, repeatable capabilities |
| **File Location** | `.claude/agents/name.md` | `.claude/skills/name/SKILL.md` |

**Key Difference**: Subagents run in separate context windows with guaranteed invocation. Skills run in the main conversation and activate automatically when Claude detects relevance.

**Use skills when**: Task is simple, repeatable, and doesn't need context isolation (blog planning, PDF extraction, note organizing)

**Use subagents when**: Task is complex, needs guaranteed execution, or requires separate context (multi-step refactoring, comprehensive audits)

#### 💬 AI Colearning Prompt
> "Explain the tradeoff between skills (automatic activation) and subagents (guaranteed invocation with isolated context). When would you choose one over the other?"

---

## How Skills Work: The Three-Level Architecture

Skills use a three-level loading system:

**Level 1: Brief Summary (Always Loaded)**
When Claude Code starts, it sees short descriptions of all available skills. This helps Claude know WHEN to use each skill without loading full instructions.

**Level 2: Full Instructions (On-Demand)**
When Claude decides a skill is relevant, it loads the complete SKILL.md file with detailed instructions, workflows, and examples.

**Level 3: Supporting Files (If Needed)**
Skills can bundle scripts, reference docs, or tools in their directory. Claude accesses these when executing the skill.

**Example Structure**:
```
.claude/skills/pdf-skill/
├── SKILL.md                 # Main instructions
├── scripts/
│   └── pdf_extractor.py     # Python extraction tool
└── reference/
    └── pdf-standards.md     # Technical specs
```

**For your first skill**: Focus on Level 2 (SKILL.md with clear instructions). Add Level 3 (supporting files) only if your skill needs external tools or reference material.

---

## When Claude Code Invokes Skills Automatically

Three patterns trigger skill activation:

1. **Content Type Recognition**: Upload PDF → pdf-skill activates
2. **Task Request Recognition**: "Write a blog post" → blog-writer-skill activates
3. **Explicit Request**: "Use the blog-writer skill" → Direct activation

**To see available skills**: Ask Claude "What skills do you have?" in any session. Skills are discovered through conversation, not system commands.

#### 🎓 Expert Insight
> In AI-native development, skills encode reasoning patterns, not just commands. You're not teaching Claude "run this script"—you're teaching "when you see THIS context, apply THAT framework." This makes organizational intelligence transferable across projects and teams.

---

## Hands-On: Create Your First Custom Skill

Let's create a blog planning skill.

### Step 1: Create Directory Structure

```bash
mkdir -p .claude/skills/blog-planner
```

### Step 2: Create SKILL.md File

Create `.claude/skills/blog-planner/SKILL.md`:

```markdown
---
name: "blog-planner"
description: "Help plan engaging blog posts: research topics, create outlines, suggest headlines, and draft compelling introductions. Use when user asks to plan or write blog content."
version: "1.0.0"
---

# Blog Planning Skill

## When to Use This Skill

- User asks to "plan a blog post" or "write an article"
- User mentions blog topics, headlines, or content strategy
- User needs help structuring written content

## How This Skill Works

1. **Research the topic**: Understand the subject and target audience
2. **Create outline**: Break topic into 3-5 main sections
3. **Suggest headlines**: Provide 5 compelling headline options
4. **Draft introduction**: Write an engaging first paragraph that hooks readers

## Output Format

Provide:
- **Topic Summary**: 2-3 sentence overview
- **Target Audience**: Who should read this?
- **Outline**: Numbered list of main sections
- **Headline Options**: 5 variations (descriptive, curiosity-driven, benefit-focused)
- **Introduction Draft**: 1-2 paragraph hook

## Example

**Input**: "Help me plan a blog post about sustainable living"

**Output**:
- **Topic Summary**: Practical sustainable living tips for busy professionals
- **Target Audience**: Working adults wanting eco-friendly lifestyle changes
- **Outline**:
  1. Why sustainable living matters now
  2. 5 easy swaps you can make today
  3. Long-term sustainable habits
  4. Common myths debunked
  5. Resources for deeper learning
- **Headlines**:
  1. "5 Sustainable Living Changes You Can Make This Weekend"
  2. "Busy Professional's Guide to Eco-Friendly Living"
  3. "Sustainable Living: Easier Than You Think"
- **Introduction**: "You care about the environment, but who has time for complicated lifestyle changes? Good news: sustainable living doesn't require upending your entire routine. These five simple swaps take less than an hour to implement—and they'll cut your environmental impact by 30%."
```

### Step 3: Test Your Skill

Start Claude Code (`claude`), then ask:
```
Help me plan a blog post about learning AI tools
```

Claude recognizes "blog post" trigger, loads the skill, and applies its workflow automatically.

#### 🤝 Practice Exercise

> **Ask your AI**: "I just created a blog-planner skill. Help me test it: suggest 3 blog topics related to [your interest area]. Then explain which parts of the skill activated and how Claude knew to use it."

**Expected Outcome**: You'll understand how skill descriptions trigger activation and see the skill in action with your own content.

### Step 4: Refine Your Skill Through Co-Learning

Ask Claude to review your skill:
```
Review the blog-planner skill. What could be improved?
Suggest 2-3 specific enhancements.
```

**AI as Teacher**: Claude suggests improvements (better descriptions, additional sections)
**You as Teacher**: You specify your constraints ("headlines must be curiosity-driven, not clickbait")
**Convergence**: Together you refine the skill to match YOUR workflow

Test the updated skill to validate improvements.

---

## More Skill Ideas for Practice

Apply the same pattern to create these skills:

**Meeting Notes Organizer**:
```yaml
---
name: "meeting-notes-organizer"
description: "Transform messy meeting notes into structured summaries with action items, decisions, and follow-ups. Use when user shares meeting notes or transcripts."
---
```

**Learning Path Designer**:
```yaml
---
name: "learning-path-designer"
description: "Create structured learning plans for any topic with progressive difficulty, resource recommendations, and practice exercises. Use when user wants to learn a new subject."
---
```

**Code Review Skill**:
```yaml
---
name: "code-reviewer"
description: "Perform systematic code reviews checking security, performance, maintainability, and best practices. Use when user asks to review code."
---
```

Try creating one of these skills using the blog-planner template as your guide.

---

## Why This Matters: Reusable Organizational Capability

**Workflow Impact**: Skills transform one-time solutions into persistent organizational intelligence. Solve a problem once (code review pattern, documentation style, testing strategy), encode it as a skill, and your entire team benefits automatically.

**Paradigm Connection**: This is intelligence accumulation in action. Unlike code libraries (reuse implementation), skills reuse *reasoning patterns*. The "how to think about X" becomes transferable across projects.

**Real-World Application**: Production teams create skills for domain-specific code reviews (security, performance), architectural pattern enforcement (API design, error handling), and documentation standards.

**Link to Capstone**: In Lesson 9, you'll see how plugins bundle skills—turning your custom reasoning patterns into shareable marketplace capabilities.

---

## Example: How a Skill Works in Practice

**Scenario**: You need to extract invoice data from a PDF.

**What You Do**:
```
Extract these fields from invoice.pdf:
- Invoice number
- Date
- Total amount
- Vendor name
```

**What Happens (Behind the Scenes)**:
1. Claude reads system prompt: "pdf-skill available for PDF extraction"
2. Recognizes PDF context + extraction request
3. Loads full SKILL.md with extraction instructions
4. Executes the skill's workflow automatically
5. Returns structured data

**What You See**:
```
Extracted Invoice Details:
- Invoice #: INV-2024-00531
- Date: November 13, 2024
- Total: $2,450.00
- Vendor: Tech Solutions Inc.
```

You described what you wanted. Claude discovered the right skill and applied it automatically—no explicit commands needed.

---

## When to Use Skills vs Subagents vs Main Conversation

**Use Skills When**:
- Task is predictable and repeatable (PDF extraction, blog planning, note organizing)
- You want automatic application without explicit invocation
- Multiple similar tasks in a session

**Use Subagents When**:
- Task is complex with many context-sensitive variables
- You need guaranteed invocation and isolated context
- Task requires specialized, multi-step workflows

**Use Main Conversation When**:
- One-off, exploratory work
- Learning something new
- No specialized capability exists yet

---

## Reflection: From Commands to Intentions

Think about this paradigm shift:

**Traditional Development**:
- You know a command exists
- You type it explicitly: `pdf_extract --input file.pdf --output json`
- You manage tool invocation manually

**AI-Native Development with Skills**:
- You describe what you want: "Extract invoice data from this PDF"
- Claude discovers the right skill automatically
- The system handles tool invocation

This shift—from "command what exists" to "describe what you want"—is fundamental to AI-native development.

**Key Insight**: Skills don't just automate tasks. They encode *reasoning patterns* that make AI assistants smarter by default. When you create a skill, you're teaching Claude "how to think about" a domain, not just "what commands to run."

---

## Try With AI

**Setup**: Open Claude Code (`claude` command) for this activity.

**Prompt 1: Design a Custom Skill**
```
Think about a repeated task in your workflow (code reviews, meeting notes, documentation).
Design a skill for it:
- What's the skill name and description?
- When should Claude recognize it's relevant?
- What workflow should it follow?
Create the SKILL.md file structure.
```

**Prompt 2: Skill vs Subagent Decision**
```
Compare these two tasks:
Task A: Draft weekly blog posts (happens 3x/week)
Task B: Create comprehensive product launch strategy (one-time, complex)

For each: Should I build a Skill or Subagent? Why?
What's the tradeoff between automatic activation (skills) vs. guaranteed control (subagents)?
```

**Expected Outcomes**: You'll design a skill matching your workflow and understand when skills vs subagents apply.

**Best Practices Reference**: See Lesson 9 for comprehensive skill design patterns and common pitfalls.
