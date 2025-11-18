---
description: Template for part-level README.md files - standardized structure for all book parts
---

# Part README.md Template: Standardized Structure

Use this template when creating or updating **Part-level README.md files** (one per Part folder, NN-Part-Name/). This ensures consistent structure, tone, and reader experience across all parts of the book for international publication.

---

## File Location

```
book-source/docs/
└── NN-Part-Name/
    ├── README.md        ← This file (MUST be uppercase)
    ├── NN-chapter-name/
    │   ├── README.md
    │   └── NN-lesson-1
```

---

## Template Structure (11 Sections)

```yaml
---
sidebar_position: N
title: "Part N: Part Title"
---

# Part N: Part Title

## [Opening Hook - Engaging Story/Scenario]

[2-3 paragraph narrative that opens with a concrete story, example, or thought-provoking scenario. This is NOT a dry introduction—it should capture attention immediately and establish why this part matters.]

[Transition sentence connecting the hook to the rest of the part: "This part solves that problem" or "Here's what's different about Part N"]

---

## What Makes This Part Different

[2-3 paragraphs explaining the distinctive characteristics, philosophy, or approach of this part. Contrast with previous parts if applicable. Establish the "why" behind the content.]

**For example:**
- Part 1 was conceptual; Part 2 is hands-on
- Part 3 focuses on communication; Part 4 focuses on implementation
- This part emphasizes [specific methodology/skill/mindset]

---

## Who This Part Is For

[Use bold subheadings for distinct audience segments. Each segment should have 1-2 sentences explaining how this part serves that audience.]

**If you're a [persona 1]** [specific benefit and approach]

**If you're a [persona 2]** [specific benefit and approach]

**If you're a [persona 3]** [specific benefit and approach]

**If you're a [persona 4]** [specific benefit and approach]

---

## What You'll Learn in Part N

[Introductory sentence explaining the scope: "This part consists of X interconnected chapters that..."]

### Chapter N: [Chapter Title]

**Estimated time: X-Y hours**

[1-2 paragraph narrative description of what this chapter covers, why it matters, and what makes it special. NOT just a list—tell a story about what they'll learn and why it's valuable.]

**What you'll accomplish**:
- ✅ [Concrete accomplishment 1]
- ✅ [Concrete accomplishment 2]
- ✅ [Concrete accomplishment 3]
- ✅ [Concrete accomplishment N]

---

### Chapter N+1: [Chapter Title]

[Repeat the above structure for each chapter in the part]

---

## What You Won't Learn (Yet)

**[1-2 sentences establishing scope boundaries]**

**Specifically, you won't learn:**
- [Out-of-scope topic 1]
- [Out-of-scope topic 2]
- [Out-of-scope topic 3]

**What you will get** is [specific value/capability]: [2-3 sentences explaining what IS included and why these boundaries matter]

---

## How to Read This Part

**[Opening guidance on approach specific to this part's type]**

[Use bold subheadings for distinct reading strategies or guidance points, adapted to part content]

**[Strategy 1 - e.g., "Take it slow and hands-on" for practical parts]** [Specific guidance for this part]

**[Strategy 2 - e.g., "Follow the scaffolding" for parts with progressive complexity]** [Specific guidance]

**[Strategy 3 - e.g., "Expect confusion, then clarity" for challenging topics]** [Specific guidance]

**[Strategy N]** [Guidance specific to this part's unique needs]

---

## What Comes After Part N

[2-3 paragraphs explaining the transition to the next part, how this part connects to subsequent learning, and what becomes possible after completing this part.]

[Show forward momentum: "Once you complete... you'll be ready for... You'll have [specific skills/knowledge] in place—the foundation for..."]

[Include brief preview of Part N+1 if helpful]

---

## A Note on Mindset

[1-3 paragraphs addressing the psychological/philosophical dimension of this part's content. NOT just technical—address how learners should think about the material, common misconceptions, or the perspective shift required.]

**For example:**
- Balance between reliance and independence
- Shifting from memorization to understanding
- Embracing iteration and experimentation
- From isolation to collaboration

[Use concrete examples or scenarios to illustrate the mindset shift]

---

## Prerequisites

**You absolutely need:**
- ✅ [Required prerequisite 1 - usually previous part]
- ✅ [Required prerequisite 2]
- ✅ [Required prerequisite 3]

**You don't need:**
- ❌ [Non-required item 1]
- ❌ [Non-required item 2]
- ❌ [Non-required item 3]

**Special notes:** [If applicable - platform requirements, account setup, corporate considerations, etc.]

---

## Ready?

[1-2 paragraphs creating momentum and excitement. Summarize the transformation that will occur by completing this part. This is the emotional/motivational closing.]

[Concrete statement of what becomes possible after this part]

[Final call-to-action: "Let's [action]"]

---

**Part N Contents:**

1. [Chapter N: Chapter Title](#)
2. [Chapter N+1: Chapter Title](#)
3. [Chapter N+2: Chapter Title](#)
[... for all chapters in the part]

---
```

---

## Critical Rules for Part READMEs

### 1. **Narrative, Not Specification**
- ❌ NEVER: "Purpose: Gain hands-on experience with..."
- ✅ ALWAYS: "Picture this: You've just finished Part 1..."
- Part READMEs tell a story; they don't list features

### 2. **Opening Hook (Required)**
- Must start with a concrete scenario, story, or thought-provoking question
- Should NOT be an abstract overview
- 2-3 paragraphs minimum
- Examples:
  - Part 1: "A solo developer in Seattle launches a financial analysis tool..."
  - Part 2: "You open your terminal... and you're staring at a blank screen..."

### 3. **Audience Segmentation (Required)**
- Use **bold subheadings** for distinct personas
- Each persona gets 1-2 sentences explaining how this part serves them
- Include 4-5 different audience types when possible
- Examples: "If you're a complete beginner...", "If you're an experienced developer...", "If you're skeptical..."

### 4. **Chapter Descriptions (Not Lists)**
- ✅ Each chapter as H3 subheading with narrative description
- ✅ Include estimated time clearly
- ✅ 1-2 paragraphs of context/why it matters (not just topics)
- ✅ Bulleted accomplishments with checkmarks
- ❌ NEVER: Simple bullet lists of topics

### 5. **"What You Won't Learn (Yet)" (Required)**
- Establishes clear boundaries
- Explains what IS included and why
- Example: "This part is hands-on and operational, not theoretical"

### 6. **"How to Read This Part" (Required)**
- Provides SPECIFIC guidance for this part's type
- Different guidance for conceptual vs. technical vs. practical parts
- Use bold subheadings for each strategy
- Examples:
  - Conceptual: "Take your time... Pause and think..."
  - Technical: "Follow along... Make mistakes safely..."
  - Hands-on: "Expect confusion... Use AI tools..."

### 7. **Mindset Section (Required)**
- Addresses psychological/philosophical dimension
- NOT just content overview
- Discusses perspective shifts, common misconceptions, or learning approach
- Concrete examples, not abstract philosophy

### 8. **Consistent Tone**
- Engaging, conversational, reader-focused
- Use "you" and "your" throughout
- Active voice
- Professional yet approachable
- Same tone as lesson.md output style

### 9. **Prerequisites Section (Required)**
- Use checkmarks and X marks for clarity
- Separate "absolutely need" from "don't need"
- Include special notes (platform requirements, accounts, corporate considerations)

### 10. **Closing "Ready?" Section (Required)**
- Creates momentum and emotional engagement
- Summarizes transformation that will occur
- Concrete statement of what becomes possible
- Final call-to-action

### 11. **Footer Chapter List (Required)**
- List all chapters with their titles
- Use markdown link format: `[Chapter N: Title](#)`
- Helps with navigation and gives overview

---

## Frontmatter Specification

```yaml
---
sidebar_position: N           # Sequential: Part 1 = 0, Part 2 = 2, Part 3 = 3, etc.
title: "Part N: Part Title"   # MUST match H1 heading exactly
---
```

---

## Section Word Counts (Guidelines)

| Section | Words | Type |
|---------|-------|------|
| Opening Hook | 400-600 | Narrative |
| What Makes This Part Different | 300-500 | Context |
| Who This Part Is For | 400-600 | Audience |
| What You'll Learn | 1500-2000 | Chapter descriptions |
| What You Won't Learn | 200-300 | Boundaries |
| How to Read | 400-600 | Guidance |
| What Comes After | 300-500 | Forward bridge |
| A Note on Mindset | 300-600 | Philosophy |
| Prerequisites | 150-250 | Requirements |
| Ready? | 200-300 | Closing |
| **TOTAL** | **4500-6500** | Essay format |

---

## Content Enrichment Patterns (Apply These Throughout)

See lesson.md for detailed patterns, but for Parts:

1. **Rich Storytelling**: Include 3-5 concrete examples throughout (not just opening)
2. **Historical Context**: Reference precedents or patterns (2-3 comparisons)
3. **Personal Relevance**: Show how content applies to reader's specific situation
4. **Honest Assessment**: Address doubts proactively with evidence
5. **Forward Momentum**: Strong transitions showing how parts connect

---

## Examples of Strong Opening Hooks

### ✅ Part 1 (Conceptual)
> "Picture this: A solo developer in Seattle launches a financial analysis tool specifically for small accounting firms. Within 18 months, the application serves 50,000 users and generates $12 million in annual revenue..."

### ✅ Part 2 (Hands-on)
> "You open your terminal (or maybe you've never opened one before), and you're staring at a blank screen. You've heard about Claude Code, Gemini CLI, and GitHub. You know they're powerful. You have no idea where to start..."

### ❌ Weak Opening
> "Purpose: Become proficient with the AI development tools..."

### ❌ Weak Opening
> "Overview: This part introduces..."

---

## Examples of Strong Chapter Descriptions

### ✅ Narrative with Purpose
> "You'll install and configure Claude Code, understanding why this 'simple' command-line interface triggered such rapid adoption. We explore the origin story (how an internal Anthropic tool became a public phenomenon), walk through installation on Windows, Mac, and Linux, and demonstrate Claude Code's unique architecture..."

### ❌ List-Based
> "- Installation steps
> - Subagents
> - Agent skills
> - MCP servers"

---

## Validation Checklist

- [ ] File named `README.md` (uppercase)
- [ ] YAML frontmatter includes sidebar_position and title
- [ ] Opening hook is narrative (story/scenario), not abstract
- [ ] "What Makes This Part Different" section present and contextual
- [ ] "Who This Part Is For" includes 4+ distinct audience segments with bold subheadings
- [ ] Chapter descriptions use H3 subheadings with narrative (not lists)
- [ ] Each chapter includes estimated time
- [ ] Each chapter includes bulleted "What you'll accomplish" with checkmarks
- [ ] "What You Won't Learn (Yet)" section clearly boundaries scope
- [ ] "How to Read This Part" provides specific guidance adapted to part type
- [ ] "A Note on Mindset" addresses psychological/philosophical dimension
- [ ] Prerequisites section includes both "absolutely need" and "don't need"
- [ ] "Ready?" closing creates momentum and includes call-to-action
- [ ] Footer includes chapter list with proper formatting
- [ ] Tone is engaging, conversational, reader-focused throughout
- [ ] All sections use active voice and direct address ("you")
- [ ] No typos or grammatical errors
- [ ] Approximately 4500-6500 words (essay format, not condensed)
- [ ] Professional publication quality
- [ ] Matches structure and tone of Part 1 and Part 2 READMEs

---

## Quick Reference: The 11 Sections in Order

1. **Opening Hook** — Engaging story/scenario (2-3 paragraphs)
2. **What Makes This Part Different** — Distinctive characteristics (2-3 paragraphs)
3. **Who This Part Is For** — Audience segments with bold subheadings
4. **What You'll Learn** — Chapter-by-chapter descriptions (H3 + narrative)
5. **What You Won't Learn (Yet)** — Clear scope boundaries
6. **How to Read This Part** — Specific guidance for this part's type
7. **What Comes After** — Transition to next part (2-3 paragraphs)
8. **A Note on Mindset** — Psychological/philosophical framing
9. **Prerequisites** — Required and not-required items
10. **Ready?** — Momentum-building closing
11. **Footer** — Chapter list and divider

---

## How This Ensures International Book Standards

This template enforces:

✅ **Consistency** — All parts follow identical structure
✅ **Readability** — Narrative format easier to translate
✅ **Engagement** — Opening hooks and mindset sections maintain reader interest
✅ **Clarity** — Scope boundaries prevent confusion
✅ **Professionalism** — Publication-quality throughout
✅ **Accessibility** — Grade 7+ reading level, active voice, direct address
✅ **Scaffolding** — Progressive complexity visible across parts

---

## Using This Template

1. **Copy the template structure** above
2. **Adapt to your specific part** (different hooks, audiences, chapters, mindset focus)
3. **Preserve the 11-section order** (don't reorder or rename sections)
4. **Follow all critical rules** (no exceptions)
5. **Maintain tone and style** matching lesson.md and existing parts
6. **Validate against checklist** before submitting
7. **Ensure ~4500-6500 words** (essay format, not condensed)

---

## Support & Questions

This template ensures all part READMEs maintain the standardized structure exemplified by:
- Part 1: Introducing AI-Driven Development
- Part 2: AI Tool Landscape

Use these as reference implementations for tone, structure, and depth.

