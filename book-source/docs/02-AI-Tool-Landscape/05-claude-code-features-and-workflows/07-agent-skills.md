---
title: "Agent Skills"
sidebar_position: 7
chapter: 5
lesson: 7
duration_minutes: 12

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
  new_concepts: 9
  assessment: "9 concepts (skills definition, three-level loading, autonomous discovery, SKILL.md structure, skills vs subagents, intelligence accumulation, co-learning refinement, skill types, skill creation rules) - within B1 limit of 10 ✓"

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
version: "2.1.0"

# Legacy compatibility
prerequisites:
  - "Lessons 2-5: Claude Code, CLAUDE.md, MCP, Subagents"
  - "Understanding of reusable components"
---

# Agent Skills: Teaching Claude New Capabilities

## The Problem: Smart But Not Expert

Meet **Dr. Claude** – a brilliant doctor who graduated top of their class. They know *everything* about medicine in general. But here's the problem:

> **Patient**: "Doctor, I need brain surgery."
>
> **Dr. Claude**: "I understand what brain surgery is! I've read every textbook. But... I've never actually *done* one. I don't know where your hospital keeps the special tools. I don't know your hospital's specific procedures."

**Dr. Claude is intelligent but lacks specialized, practical knowledge.**

This is exactly what happens when you work with Claude Code on repeated tasks. You're working on multiple blog posts this week. Every time you start a new session, you give the same instructions:

"Create an outline with 5 main sections, suggest 5 headline variations, make the introduction hook readers in the first sentence, keep paragraphs under 4 sentences..."

By the third blog post, you're frustrated. **Why can't Claude just remember how you like blog posts structured?**

You could use a subagent—but that requires explicitly saying "Use the blog-planner subagent" every time. What if Claude could **automatically** apply your blog-planning workflow whenever you mention writing a blog post?

**That's what Skills solve.**

---

## What Are Agent Skills?

> **Simple Definition**: A Skill is a folder of instructions that teaches Claude Code how to do something specific really well.

**Think of it like this:**

| Without Skill | With Skill |
| --- | --- |
| A chef who knows cooking theory | A chef with a detailed recipe book |
| A driver who knows traffic rules | A driver with GPS navigation |
| A student who knows math concepts | A student with step-by-step problem guides |

**Key characteristic**: Skills are **discovered autonomously**. You create the skill once (SKILL.md file), and Claude applies it automatically when relevant—no explicit invocation needed.

### The Recipe Book Analogy

Imagine you're a **brilliant chef** who understands all cooking techniques. But someone asks you to make "Grandma's Secret Biryani."

**Without the recipe (No Skill):**
- You might make *a* biryani
- It won't taste like Grandma's
- You'll guess the spices
- You might make mistakes

**With the recipe (With Skill):**
- You follow exact steps
- You use the right spices in the right amounts
- The result is consistent every time
- It tastes exactly like Grandma's!

**A Skill is like giving Claude Code the perfect recipe for a specific task.**

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

#### AI Colearning Prompt
> "Explain the tradeoff between skills (automatic activation) and subagents (guaranteed invocation with isolated context). When would you choose one over the other?"

---

## How Skills Work: The Three-Level Architecture

Here's something clever about Skills: **Claude doesn't read all Skills at once.**

Why? Because Claude has a **limited memory** (called "context window"). If it loaded every skill at the start, it would run out of space!

### The Phone Apps Analogy

Think of your smartphone:
- You have **100 apps** installed
- Your phone doesn't run all 100 at once (it would crash!)
- Apps stay **closed** until you tap on them
- When you need one, it **opens and loads**

Skills work the same way:
- Claude has access to many skills
- They stay "closed" until needed
- When Claude needs one, it "opens" and reads the instructions

### The Three Levels

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

## What's Inside a Skill?

### Skills are Just Folders!

This is the beautiful part – Skills are incredibly simple. They're just **folders with files** inside.

```
📁 my-skill/
│
├── 📄 SKILL.md          ← REQUIRED: The main instructions
│
├── 📁 scripts/          ← OPTIONAL: Helper code
│   └── helper.py
│
├── 📁 references/       ← OPTIONAL: Documentation
│   └── guide.md
│
└── 📁 assets/           ← OPTIONAL: Templates, images
    └── template.pptx
```

The most important file is **SKILL.md** – it contains the instructions Claude reads.

### The IKEA Furniture Analogy

Think of buying IKEA furniture:

| IKEA Furniture | Claude Code Skill |
| --- | --- |
| The box | The skill folder |
| Instruction manual | SKILL.md file |
| Included tools | Helper scripts |
| Picture of final result | Templates/examples |

When you open the box, you read the manual and follow the steps. Same with Skills – Claude reads SKILL.md and follows those instructions!

### Anatomy of SKILL.md

Every SKILL.md has two parts:

**Part 1: Frontmatter (The ID Card)**

```yaml
---
name: my-skill
description: What this skill does and WHEN to use it
version: "1.0.0"
---
```

**Part 2: Body (The Instructions)**

```markdown
# My Skill Name

## Overview
What this skill helps with...

## How to Use
Step-by-step instructions...
```

---

## When Claude Code Invokes Skills Automatically

Three patterns trigger skill activation:

1. **Content Type Recognition**: Upload PDF → pdf-skill activates
2. **Task Request Recognition**: "Write a blog post" → blog-writer-skill activates
3. **Explicit Request**: "Use the blog-writer skill" → Direct activation

**To see available skills**: Ask Claude "What skills do you have?" in any session. Skills are discovered through conversation, not system commands.

#### Expert Insight
> In AI-native development, skills encode reasoning patterns, not just commands. You're not teaching Claude "run this script"—you're teaching "when you see THIS context, apply THAT framework." This makes organizational intelligence transferable across projects and teams.

---

## Types of Skills

### 1. Foundational Skills

**What:** Basic capabilities that make Claude useful for common tasks

**Examples:**
- Creating Word documents (.docx)
- Making PowerPoint presentations (.pptx)
- Working with Excel spreadsheets (.xlsx)
- Creating PDFs

**Analogy:** These are like **basic life skills** – cooking, cleaning, driving. Everyone needs them!

### 2. Third-Party Skills

**What:** Skills that help Claude work with specific software or services

**Examples:**
- Navigating websites with browser tools
- Understanding a Notion workspace
- Working with specific APIs

**Analogy:** These are like **specialized certifications** – learning to use Photoshop, getting a forklift license, or becoming certified in a specific software.

### 3. Enterprise/Custom Skills

**What:** Skills created by organizations for their specific needs

**Examples:**
- Your company's coding style guide
- Internal documentation standards
- Organization-specific workflows

**Analogy:** These are like **company training manuals** – every company has their own way of doing things!

---

## Skill Creation Rules

Before creating skills, remember these guidelines:

### DO

- **Be concise** - Claude is smart, don't over-explain
- **Use clear examples** - Show, don't just tell
- **Write a good description** - This triggers the skill
- **Focus on what Claude doesn't know** - Your specific preferences

### DON'T

- Create README files (unnecessary)
- Include installation guides (Claude doesn't need them)
- Write verbose explanations (wastes context space)
- Add irrelevant information (keep it focused)

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

#### Practice Exercise

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

## Bonus Example: Study Notes Creator Skill

Here's another practical skill example for students:

**Use Case**: You always want notes in a specific format – summary at top, key terms highlighted, questions at the end.

Create `.claude/skills/study-notes-creator/SKILL.md`:

```markdown
---
name: "study-notes-creator"
description: "Creates well-organized study notes from any content. Use when the user asks for study notes, learning summaries, revision materials, or wants to understand a topic for studying."
version: "1.0.0"
---

# Study Notes Creator

## Overview
This skill creates structured study notes that help students learn and retain information effectively.

## Output Format
Every study note should follow this structure:

### 1. Summary Section
- Start with a 2-3 sentence overview
- Highlight the main concept in bold

### 2. Key Terms
- List 5-10 important vocabulary words
- Format: **Term** - Definition

### 3. Main Content
- Use bullet points for easy scanning
- Include examples for complex ideas
- Keep paragraphs short (3-4 sentences max)

### 4. Practice Questions
- Include 3-5 questions at the end
- Mix of recall and understanding questions

### 5. Quick Review Box
- End with a "Quick Review" box
- 3-5 bullet points summarizing key takeaways

## Example Output

**Topic: Photosynthesis**

**Summary**
Photosynthesis is the process plants use to convert sunlight into food. It's essential for life on Earth.

**Key Terms**
- **Chlorophyll** - Green pigment that absorbs light
- **Glucose** - Sugar produced by photosynthesis
- **Carbon dioxide** - Gas absorbed from air

**Main Content**
[Content here...]

**Practice Questions**
1. What is the main purpose of photosynthesis?
2. Where does photosynthesis occur in a plant?

**Quick Review**
- Plants make their own food
- Requires sunlight, water, CO2
- Produces glucose and oxygen
```

---

## More Skill Ideas for Practice

Apply the same pattern to create these skills:

**Meeting Notes Organizer**:
```yaml
---
name: "meeting-notes-organizer"
description: "Transform messy meeting notes into structured summaries with action items, decisions, and follow-ups. Use when user shares meeting notes or transcripts."
version: "1.0.0"
---
```

**Learning Path Designer**:
```yaml
---
name: "learning-path-designer"
description: "Create structured learning plans for any topic with progressive difficulty, resource recommendations, and practice exercises. Use when user wants to learn a new subject."
version: "1.0.0"
---
```

**Code Review Skill**:
```yaml
---
name: "code-reviewer"
description: "Perform systematic code reviews checking security, performance, maintainability, and best practices. Use when user asks to review code."
version: "1.0.0"
---
```

Try creating one of these skills using the blog-planner template as your guide.

---

## Why This Matters: Reusable Organizational Capability

### The Growing Garden Analogy

Think of Claude Code as a **garden**:

| Garden Element | Claude Code Equivalent |
| --- | --- |
| The soil | Claude's base intelligence |
| Seeds | Skills |
| Water & sunlight | Your instructions |
| Flowers/vegetables | The output/results |

**More skills = A richer, more capable garden!**

### Visual Summary

```
┌─────────────────────────────────────────────┐
│              CLAUDE CODE                    │
│         (The Brilliant Brain)              │
│                   +                         │
│    ┌─────────┐ ┌─────────┐ ┌─────────┐    │
│    │ Skill 1 │ │ Skill 2 │ │ Skill 3 │    │
│    │  docx   │ │  pptx   │ │   pdf   │    │
│    └─────────┘ └─────────┘ └─────────┘    │
│                   =                         │
│         POWERFUL AI ASSISTANT              │
│    (Smart + Specialized Knowledge)         │
└─────────────────────────────────────────────┘
```

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

## Key Takeaways

Remember these 5 points:

1. **Skills are folders** – Simple collections of files with instructions
2. **Skills add expertise** – They give Claude specialized knowledge it doesn't have by default
3. **Skills are loaded when needed** – Not everything loads at once (saves memory!)
4. **Anyone can create Skills** – Even you, once you learn!
5. **Skills make Claude better** – The more skills, the more capable the agent

> **Summary**: Skills are instruction folders that transform Claude from a brilliant generalist into a specialized expert for specific tasks.

*"Claude is smart. Skills make it wise."*

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

Let's design reusable skills that extend Claude Code's capabilities for your specific workflow.

**Practice Designing a Custom Skill:**

> "I have a repeated task in my workflow: [describe your task: code reviews / blog drafting / meeting notes / API documentation]. Help me design an agent skill for it. Walk me through: What should the skill name and description be? When should Claude recognize it's relevant and auto-load it? What workflow should the skill follow? Create the complete SKILL.md file structure with YAML frontmatter and instructions."

**Understand Skills vs Subagents:**

> "Compare these two tasks: Task A) Draft weekly blog posts (happens 3x/week). Task B) Create comprehensive product launch strategy (one-time, complex). For each task, should I build a Skill or a Subagent? Explain the tradeoff between automatic activation (skills discover themselves) vs. guaranteed control (subagents invoked explicitly)."

**Test Skill Discovery:**

> "I just created a [your skill name] skill and saved it to `.claude/skills/`. Help me test if Claude Code auto-loads it correctly. Give me a prompt that should trigger this skill's activation. Then explain how I can verify the skill is being used vs. Claude responding without the skill."

**Build a Skill Suite:**

> "I work on [describe your project type: web apps / data pipelines / documentation / etc.]. Help me identify 3-5 reusable skills I should create that would improve my workflow. For each skill, explain: what it does, when it activates, and why it's better as a skill than as a subagent or direct prompting."

---

## Further Resources

Visit Anthropic's official skills repository to see real-world skill examples:

https://github.com/anthropics/skills
