---
description: Standardized template for chapter-level README.md files across all parts and chapter types
---

# Chapter README.md Template: Standardized Structure

Use this template when creating or updating **chapter-level README.md files**. This ensures consistent structure and professional quality across all chapters in the book, whether they are Conceptual (Part 1), Practical (Parts 2-3), or Technical (Parts 4+).

---

## File Location

```
book-source/docs/
└── NN-Part-Name/
    └── NN-chapter-name/
        ├── README.md        ← This file (MUST be uppercase)
        ├── 01-lesson-name.md
        ├── 02-lesson-name.md
        └── ...
```

## Template Structure

```yaml
---
sidebar_position: N
title: "Chapter N: Chapter Title"
---

# Chapter N: Chapter Title

[Direct connection to previous chapter: "In Chapter X, you learned/discovered Y..."]

[Transition statement or provocative question that establishes what this chapter addresses]

[1-2 compelling paragraphs establishing scope, approach, and what makes this chapter important. Mention the number of sections and preview key concepts without listing them all.]

[Final paragraph establishing tone and expectations - conceptual/strategic vs. technical/practical]

## What You'll Learn

By the end of this chapter, you'll understand:

- **[Topic/Framework name]**: [Detailed description with specifics, examples, or concrete details explaining what this topic covers and why it matters]
- **[Topic/Framework name]**: [Detailed description - should be 1-2 sentences providing rich context]
- **[Topic/Framework name]**: [Detailed description with specific examples or outcomes]
- **[Topic/Framework name]**: [Detailed description - aim for 4-6 bullet points total]
- **[Topic/Framework name]**: [Detailed description]
- **[Your strategic positioning / Your path forward / etc.]**: [Final bullet that personalizes learning to the reader's situation]
```

## Template Guidance

### Section: Opening Narrative (No Heading)

**Purpose**: Immediately engage the reader and establish the chapter's importance within the learning journey.

**What to include**:
1. **Connection to previous chapter**: Start with "In Chapter X, you learned/discovered Y..." to establish continuity
2. **Transition or provocative question**: A compelling statement or question that establishes what this chapter addresses
3. **Scope and approach**: 1-2 paragraphs describing what the chapter covers (mention number of sections), key frameworks/concepts, and approach (strategic/conceptual vs. technical/practical)
4. **Tone setting**: Final paragraph establishing whether this is conceptual/strategic or hands-on/practical

**Tone**: Direct, narrative, engaging—like storytelling, not a syllabus. Avoid headings like "Overview" or "Introduction."

**Length**: 3-5 paragraphs total

### Section: What You'll Learn

**Purpose**: Set clear, rich expectations for the chapter with detailed context.

**Requirements**:
- Use bullet points (not numbered list)
- Each bullet has two parts: **Bold topic/framework name**: Detailed description
- Descriptions should be 1-2 sentences with specific examples, concrete details, or outcomes
- Include specifics: numbers, examples, timeframes, comparisons
- 4-6 bullets total (aim for depth over quantity)
- Final bullet often personalizes to reader's situation ("Your strategic positioning", "Your path forward", etc.)

**Format**:
```
- **[Framework/Topic name]**: [1-2 sentence detailed description with specifics, examples, or concrete outcomes]
```

**Examples of rich bullets**:
- **The Snakes & Ladders framework**: Why competing in vertical markets (healthcare, legal, logistics) offers better odds than competing at the consumer layer, and how the game board has fundamentally changed
- **Super orchestrator economics**: How tiny teams generate billion-dollar value by orchestrating AI to handle the mechanical 90% while humans focus on the creative 10%—from Instagram's 13-person team to Claude Code's single-developer model

## Critical Rules

1. **File Naming**: ALWAYS use `README.md` (uppercase), NEVER `readme.md` or `index.md`
2. **Structure**: Chapters have only ONE section with a heading: **What You'll Learn** (opening narrative has NO heading)
3. **No "Overview" heading**: Start directly with narrative paragraphs connecting to previous chapter
4. **No subsections**: Do NOT include "Chapter Structure", "Prerequisites", "What Makes This Chapter Unique", "Time Investment", "How to Read This Chapter", "What's Next", etc.
5. **Learning bullets must be rich**: Each bullet should have **bold topic name**: detailed description (1-2 sentences with specifics)
6. **Narrative flow**: Write as if speaking directly to the reader; establish context and relevance like storytelling
7. **Professional Polish**: Publication-quality writing with no typos, clear prose, and engaging tone
8. **YAML order**: Always `sidebar_position` first, then `title`

## Example: Good Chapter README

```markdown
---
sidebar_position: 3
title: "Chapter 3: Mastering Your AI Pair"
---

# Chapter 3: Mastering Your AI Pair

In Chapters 1 and 2, you understood the transformation and the strategic context. Now comes the practical work: learning to *collaborate effectively* with AI.

This chapter moves beyond experimental "vibe coding" into disciplined partnership. Through seven interconnected sections, you'll discover how to set up your AI development environment, establish clear communication patterns with your AI pair, validate suggestions critically, debug collaboratively, and apply the Spec-Driven Development workflow to structure real projects.

You'll learn when to guide and when to trust, how to recognize anti-patterns before they derail your productivity, and how to restore effectiveness when AI collaboration breaks down. This is hands-on and practical—you'll configure actual tools, run real commands, and build working projects.

By the end of this chapter, you'll have working tools, proven workflows, and the confidence to build production-quality software with AI as a true collaborator.

## What You'll Learn

By the end of this chapter, you'll understand:

- **Environment setup and toolchain**: How to configure your local development environment with AI-first tools (Claude Code, Git, VS Code extensions) and verify your installation with a test project
- **Critical evaluation framework**: The three criteria for assessing AI suggestions (correctness, clarity, codebase alignment) and how to spot hallucinations or inappropriate recommendations before they reach production
- **Collaborative debugging**: How to interpret error messages, guide AI toward root causes, and iterate through solutions—treating AI as a debugging partner, not an oracle
- **Spec-Driven Development workflow**: The complete 7-step SDD process (Specify → Plan → Tasks → Implement → Test → Review → Refine) that transforms vibe coding into reliable, repeatable development
- **Anti-patterns and recovery strategies**: Five common failure modes (over-reliance, context loss, drift, premature complexity, tooling misalignment) and specific techniques to restore productivity
- **Your development identity**: How to position yourself as an active orchestrator—not a passive consumer—of AI assistance, establishing habits that compound over every future project
```

## Validation Checklist

- [ ] File named `README.md` (uppercase)
- [ ] YAML frontmatter: `sidebar_position` first, then `title`
- [ ] Opening narrative (NO "Overview" heading): 3-5 paragraphs connecting to previous chapter and establishing scope
- [ ] Opening starts with "In Chapter X, you learned/discovered Y..."
- [ ] Opening mentions number of sections and previews key concepts
- [ ] **What You'll Learn** section with 4-6 bullet points
- [ ] Each bullet has format: **Bold topic**: Detailed 1-2 sentence description with specifics
- [ ] Bullets include concrete details: numbers, examples, frameworks, outcomes
- [ ] Final bullet often personalizes to reader ("Your strategic positioning", "Your development identity", etc.)
- [ ] Tone is direct, narrative, and engaging ("you", "your", conversational storytelling)
- [ ] No typos, grammatical errors, or placeholder text
- [ ] Professional, publication-quality writing throughout
