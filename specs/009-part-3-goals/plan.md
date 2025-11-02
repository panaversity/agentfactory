# Implementation Plan: Part-3 Goals - Working with AI Coding Agents

**Branch**: `009-part-3-goals` | **Date**: 2025-01-15 | **Spec**: [specs/009-part-3-goals/spec.md](./spec.md)
**Input**: Feature specification from `/specs/009-part-3-goals/spec.md`

## Summary

Create a 400-600 word introduction for Part-3 of the book that establishes learning goals for both Chapter 9 (Prompt Engineering for AIDD) and Chapter 10 (Context Engineering for AIDD). The introduction must position these two chapters as foundational skills for AI-native development, emphasizing that learners will transition from "understanding AI concepts" (Parts 1-2) to "actively using AI agents to build software" (Part-3 onward). The content must be beginner-friendly, business-focused, and position AI as a co-learning partner while maintaining the book's agent-native philosophy.

## Technical Context

**Content Type**: Educational book content (Markdown/MDX for Docusaurus)
**Target Audience**: Complete beginners with no programming background (Part-3 = beginner tier)
**Complexity Tier**: Beginner-friendly (max 2 options, max 5 concepts/section, heavy scaffolding)
**Word Count**: 400-600 words (strict constraint for attention span management)
**Prerequisites**: Readers have completed Part-1 (Foundations) and Part-2 (AI Tool Landscape)
**Dependencies**: Chapter 9 and Chapter 10 content (already specified in context)
**Deliverable Format**: MDX file for Docusaurus integration
**Tone**: Encouraging, empowering, non-technical
**Key Constraint**: No jargon without immediate context/explanation

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle Alignment

✅ **Principle 1 (AI-First Teaching)**: Part-3 introduction positions AI agents (Claude Code, Gemini CLI) as primary tools for software development from this point forward

✅ **Principle 5 (Progressive Complexity)**: Part-3 is positioned in beginner tier (Parts 1-3) with maximum scaffolding, simple language, and limited options

✅ **Principle 8 (Accessibility)**: Content designed for complete beginners with no CS background, no assumed knowledge

✅ **Principle 9 (Show-Spec-Validate)**: Goals introduction will set expectations for specification-first pattern used throughout both chapters

✅ **Principle 12 (Cognitive Load - Beginner Tier)**:
- Max 2 options (learners choose between Claude Code OR Gemini CLI)
- Max 5 concepts introduced (prompting, context, AI agents, validation, iteration)
- Minimal/simplest explanations first
- One key skill per chapter (Ch9: prompting, Ch10: context)

✅ **Principle 13 (Concept-Before-Command)**: Introduction explains WHAT prompt engineering and context engineering are conceptually before HOW to apply them

✅ **Principle 14 (Planning-First)**: Goals emphasize that specification-writing (which requires effective prompting) is the foundation

✅ **Principle 15 (Validation-Before-Trust)**: Goals mention that learners will validate AI-generated code

### Gate Status: **PASS**

No constitution violations. Content aligns with beginner-friendly, agent-native, specification-first methodology.

## Project Structure

### Documentation (this feature)

```text
specs/009-part-3-goals/
├── spec.md              # Feature specification (completed)
├── plan.md              # This file (implementation plan)
├── content-outline.md   # Phase 0: Detailed content structure
└── tasks.md             # Phase 2: Created by /sp.tasks command
```

### Deliverable Location

```text
book/
└── docs/
    └── part-3/
        └── intro.mdx    # Part-3 introduction (400-600 words)
```

**Note**: This is educational content, not source code. No src/ or tests/ directories needed.

## Complexity Tracking

**No complexity violations.** This is a straightforward educational content piece with clear constraints and structure.

---

## Phase 0: Content Outline & Research

### Content Structure Breakdown

The Part-3 introduction will follow this structure (400-600 words total):

#### Section 1: The Hook - Why These Skills Matter (100-120 words)
**Purpose**: Capture attention and establish relevance

**Key Messages**:
- You've learned ABOUT AI (Parts 1-2), now you'll learn to USE AI to build software
- The paradigm shift: From "I write code" to "I direct AI agents to build what I specify"
- These two skills (prompting + context) are THE foundation for everything that follows

**Tone**: Exciting, empowering, transformative

**Non-Negotiables**:
- No jargon without immediate explanation
- Use "you will..." language (active, second-person)
- Frame as natural progression from Parts 1-2

---

#### Section 2: What You'll Learn (150-180 words)
**Purpose**: Set clear learning objectives for both chapters

**Chapter 9 Preview - Prompt Engineering**:
- WHAT: How to communicate effectively with AI coding agents (Claude Code, Gemini CLI)
- WHY: Clear prompts = working code; vague prompts = broken code
- HOW: Learn the 8-element AIDD prompting framework
- OUTCOME: Construct prompts that generate production-quality code 70% of the time

**Chapter 10 Preview - Context Engineering**:
- WHAT: Managing what the AI agent "knows" about your project
- WHY: Generic context = generic code; project-specific context = code that fits perfectly
- HOW: Learn progressive loading, memory files, context compression
- OUTCOME: Maintain consistency across multi-session projects

**Link Between Chapters**:
- Prompting = what you SAY to AI
- Context = what AI KNOWS when you say it
- Together they form complete AI collaboration skillset

---

#### Section 3: The Learning Path (80-100 words)
**Purpose**: Show progression and build confidence

**Key Messages**:
- Start simple: Basic prompts for simple tasks (Chapter 9 early sections)
- Build complexity: Add technical constraints, examples, iteration (Chapter 9 advanced)
- Expand awareness: Manage multi-file projects across sessions (Chapter 10)
- Master orchestration: Direct AI as your development partner (Chapter 10 advanced)

**Reassurance**:
- No programming experience required
- Learn by doing, not by memorizing
- AI agents handle the syntax; you understand the concepts

---

#### Section 4: Success Vision - What You'll Be Able to Do (70-100 words)
**Purpose**: Paint concrete picture of capabilities after Part-3

**After Completing Part-3, You Will**:
1. Construct effective prompts that get working code on first try
2. Manage AI agent context across multi-day projects
3. Iterate with AI to refine specifications and improve code
4. Validate AI-generated code for correctness and security
5. Think like an "AI orchestrator" rather than manual coder

**Bridge to Part-4**:
- With these skills mastered, you'll apply them to learn Python (Part-4)
- From here on, every chapter uses AI collaboration as default workflow

---

### Tone & Style Guidelines

**Voice**: Encouraging mentor who believes in the reader

**Language Patterns to USE**:
- "You will learn to..." (not "This chapter covers...")
- "AI as your co-pilot" or "AI as your partner" (not "AI tool")
- "Direct AI agents" (not "use AI")
- "Validate and verify" (emphasize human responsibility)

**Language Patterns to AVOID**:
- "Obviously", "simply", "just", "easy" (ableist language)
- "AI will do everything for you" (false promise)
- Technical jargon without context
- Forward references without explanation

**Analogies to Consider**:
- Prompting = giving directions to a skilled assistant
- Context = making sure your assistant knows your project history
- Validation = quality control on work delegated to others

---

### Key Concepts to Introduce (Max 5 for Beginner Tier)

1. **Prompt Engineering**: The skill of communicating clearly with AI agents
2. **Context Engineering**: Managing what information AI agents have access to
3. **AI Agents**: Intelligent tools like Claude Code and Gemini CLI (not just autocomplete)
4. **Validation**: Checking that AI-generated code is correct and safe
5. **Iteration**: Refining prompts/context when first attempt isn't perfect

**Definitions Format** (Concept-Before-Command):
- Start with non-technical analogy
- Then provide technical definition
- Then show why it matters for building software

---

### Research: Alignment with Chapter Content

**Chapter 9 Key Topics** (from context):
- 8-element AIDD prompting framework (Command, Context, Logic, Roleplay, Formatting, Technical Constraints, Examples, Questions)
- Prompt templates for common tasks
- Claude Code vs Gemini CLI prompting strategies
- Measuring prompt effectiveness

**Chapter 10 Key Topics** (from context):
- Context window basics and limitations
- Progressive context loading strategies
- Memory files and checkpoints
- Context compression techniques
- Multi-session project continuity

**Validation**: Goals introduction accurately reflects both chapter scopes ✅

---

### Dependencies & Prerequisites

**What Readers Know Coming In** (from Parts 1-2):
- AI-native development vision and philosophy
- Overview of AI development tools (Claude Code, Gemini CLI)
- Basic understanding that AI can generate code from descriptions
- Familiarity with terminal/command-line basics

**What Readers DON'T Know Yet** (taught in Part-3):
- HOW to effectively prompt AI agents
- HOW to manage AI agent context
- WHAT makes a good vs bad prompt
- HOW to iterate when AI output isn't right

---

## Phase 1: Implementation Checklist

### Content Creation Tasks

1. **Draft Introduction** (400-600 words following structure above)
2. **Validate Word Count** (strict 400-600 range)
3. **Check Cognitive Load** (max 5 concepts, beginner-appropriate language)
4. **Verify Tone** (encouraging, empowering, non-technical)
5. **Validate Links** (accurate preview of Chapter 9 and Chapter 10 content)
6. **Constitution Alignment** (principles 1, 5, 8, 12, 13 specifically)

### Quality Gates

- [ ] Word count: 400-600 ✓
- [ ] Max 5 new concepts introduced ✓
- [ ] No jargon without immediate context ✓
- [ ] Beginner-friendly tone (no "obviously", "simply") ✓
- [ ] Accurate representation of Chapter 9 content ✓
- [ ] Accurate representation of Chapter 10 content ✓
- [ ] Clear progression from Parts 1-2 ✓
- [ ] Bridge to Part-4 established ✓
- [ ] Success criteria measurable ✓
- [ ] Constitution principles aligned ✓

### Success Metrics (from Spec)

- **SC-001**: Learners can construct effective prompts 70%+ success rate
- **SC-002**: Learners manage context with 90%+ consistency
- **SC-003**: Learners describe process as "design → AI → validate"
- **SC-004**: 80% of learners feel confident to build real projects
- **SC-005**: Learners validate AI code (don't blindly trust)

**How Introduction Supports These**:
- Sets expectation that prompting is learnable skill (SC-001)
- Explains context management importance upfront (SC-002)
- Frames development as orchestration, not manual coding (SC-003)
- Paints exciting vision of capability (SC-004)
- Emphasizes validation throughout (SC-005)

---

## Implementation Notes

### Writing Approach

1. **Write in one sitting** to maintain voice consistency
2. **Read aloud** to catch awkward phrasing (accessibility check)
3. **Count words frequently** to stay within 400-600 constraint
4. **Use bullet points sparingly** (prose flows better for introductions)
5. **End with forward momentum** (reader excited to start Chapter 9)

### Common Pitfalls to Avoid

❌ **Over-promising**: "AI will write all your code perfectly"
✅ **Realistic framing**: "AI generates code from your specifications; you validate"

❌ **Intimidating**: "You must master complex prompting techniques"
✅ **Approachable**: "You'll learn simple patterns that get powerful results"

❌ **Abstract**: "Prompting is important for AI collaboration"
✅ **Concrete**: "Clear prompts mean working code on first try"

❌ **Forward references**: "We'll cover testing in Chapter 25"
✅ **Present focus**: "After Part-3, you'll apply these skills to learn Python"

---

## Risks & Mitigation

| Risk | Impact | Mitigation |
|------|--------|------------|
| Word count exceeds 600 | Cognitive overload for beginners | Strict editing; remove non-essential details |
| Content too technical | Loses non-programmer audience | Plain language review; analogy additions |
| Misalignment with chapters | False expectations | Cross-reference with Chapter 9 & 10 content |
| Boring/unmotivating | Reader skips or loses interest | Strong hook; concrete success vision |
| Over-promising outcomes | Frustration when reality doesn't match | Realistic framing with validation emphasis |

---

## Next Steps

After plan approval:

1. Run `/sp.tasks` to generate actionable task checklist
2. Write draft content following outline above
3. Validate against quality gates
4. Review for constitution alignment
5. Integrate into Docusaurus at `docs/part-3/intro.mdx`
