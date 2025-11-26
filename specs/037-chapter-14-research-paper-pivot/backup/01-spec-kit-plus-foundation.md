---
title: "Spec-Kit Plus Foundation: What You're About to Build With"
chapter: 14
lesson: 1
duration_minutes: 20
proficiency_level: "A2"
cognitive_load:
  new_concepts: 2
  assessment: "2 new concepts (Horizontal Intelligence, Vertical Intelligence) within A2 limit of 7 ✓"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Understanding Reusable Intelligence Architecture"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain how Spec-Kit Plus captures intelligence through Horizontal (ADRs/PHRs) and Vertical (Subagents) patterns"

  - name: "Distinguishing Intelligence Types"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can differentiate Horizontal Intelligence (reasoning across time) from Vertical Intelligence (delegation hierarchy)"

learning_objectives:
  - objective: "Explain how Spec-Kit Plus captures Reusable Intelligence through ADRs, PHRs, and Subagents"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Verbal explanation with concrete examples"

  - objective: "Recognize the compounding effect of intelligence accumulation across projects"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Compare Project 1 vs Project 10 intelligence availability"

differentiation:
  extension_for_advanced: "Design hypothetical intelligence library for video production; map compounding effects across 5+ projects"
  remedial_for_struggling: "Focus on concrete ADR/PHR examples from video generation project; use visual diagrams"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/chapter-14-video-generation/spec.md"
created: "2025-11-25"
last_modified: "2025-11-25"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# Spec-Kit Plus Foundation: What You're About to Build With

Before you start building AI-generated product demo videos, understand WHAT Spec-Kit Plus is and HOW it makes Reusable Intelligence practical.

By the end of this lesson, you'll understand how Spec-Kit Plus captures and reuses intelligence through its two-architecture pattern, and see exactly what intelligence artifacts you'll build in this chapter.

---

## What Is Spec-Kit Plus?

Spec-Kit Plus is an **SDD-RI framework** built around one core idea: **capture intelligence, not just deliver code**.

![Architecture diagram showing Spec-Kit-Plus workflow with Constitution, Specification, Planning, Tasks, and Implementation phases](/img/part-4/chapter-14/spec-kit-plus-architecture.png)

![Eight-phase Spec-Kit-Plus workflow showing Constitution → Specify → Clarify → Plan → Tasks → Implement → Validate → Refactor progression](/img/part-4/chapter-14/eight-phase-workflow.png)

Every feature you build generates two outputs:
1. **Working code** (the deliverable - ephemeral, could be rewritten)
2. **Reusable intelligence** (reasoning patterns, decision frameworks, AI prompts that work - permanent)

**The framework provides**:
- Templates for specifications, plans, ADRs (Architectural Decision Records), PHRs (Prompt History Records)
- Slash commands (`/sp.specify`, `/sp.plan`, `/sp.implement`) that orchestrate AI subagents
- Directory structure that separates ephemeral code from permanent intelligence

**Critical distinction**: Spec-Kit Plus is NOT an AI service. It's a methodology framework that works WITH your AI tool (Claude Code, Gemini CLI, etc.).

---

## Horizontal Intelligence: Capturing Reasoning Across Time

Horizontal Intelligence is how Spec-Kit Plus turns decisions into reusable knowledge that persists across projects.

### ADRs (Architectural Decision Records)

Document the "WHY" behind significant decisions, not just the "WHAT" you built.

**Example**:
- **Bad documentation**: "Used Playwright MCP for browser automation"
- **ADR (Horizontal Intelligence)**: "Chose Playwright MCP over Selenium because: (1) Gemini.google.com requires real browser context with session persistence, (2) MCP provides native session management, (3) tradeoff accepted: MCP learning curve vs robust video recording"

**When you create ADRs**:
- During Constitution phase (Lesson 3) - Document video generation quality standards
- During Planning phase (Lesson 6) - Explain architectural choices for Playwright + Gemini integration
- When designing skills/subagents (Lesson 9) - Justify pattern selection for `generate-video` and `upload-youtube`

**Storage**: `history/adr/` directory becomes your team's knowledge base

**Reusability**: Six months later, new team member asks "Why Playwright MCP?" → ADR answers immediately. AI agents READ ADRs to understand project context before generating code.

### PHRs (Prompt History Records)

Automatically log AI collaboration sessions, capturing what prompts work vs what fails.

**Example**:
- **Prompt A**: "Generate a product demo video" → Resulted in generic, unconvincing video
- **Prompt B**: "Generate a 30-second product demo video highlighting the pricing page conversion flow using Gemini video generation API with high-quality voiceover" → Generated compelling video with strong marketing message
- **PHR captures**: Prompt B works, Prompt A fails, reasoning logged

**When PHRs are created**:
- Automatically during `/sp.specify`, `/sp.plan`, `/sp.implement` execution
- You don't manually invoke PHRs - the system creates them

**Storage**: `history/prompts/<feature>/` directory

**Reusability**: Project 2 (authentication system) starts with Project 1's (video generation) PHRs. Your AI reads: "Last time, prompts emphasizing specific marketing objectives and quality criteria worked better than vague requests." Project 2 specifications are immediately better.

### What Makes This "Intelligence"

This isn't documentation for humans only:
- **AI agents read ADRs** to understand project context before reasoning
- **AI agents reference PHRs** to avoid repeating past mistakes
- **Intelligence accumulates** - Project 10 is dramatically faster than Project 1

**The compounding effect**:
- Project 1 (Video Generation): Create 5 ADRs + 12 PHRs (learning from scratch, figuring out Playwright MCP + Gemini integration)
- Project 2 (New Feature): Start with 5 ADRs + 12 PHRs, create 3 new ADRs + 8 new PHRs (total: 8 ADRs, 20 PHRs)
- Project 10: Start with accumulated intelligence from 9 projects, rarely repeat mistakes, new features built 3-4x faster

---

## Vertical Intelligence: Delegation Through Specialization

Vertical Intelligence is how Spec-Kit Plus distributes work to specialized AI subagents, each designed with the Persona+Questions+Principles (P+Q+P) pattern.

### The Delegation Pattern

```
YOU: "Build a system that generates product demo videos and uploads to YouTube"
  ↓
ORCHESTRATOR: Routes to Specification Subagent
  ↓
SPEC SUBAGENT: Asks clarifying questions (What's the target video length?
                What marketing message?), generates complete spec
  ↓
YOU: Review and approve
  ↓
ORCHESTRATOR: Routes to Planning Subagent
  ↓
PLAN SUBAGENT: Identifies Playwright MCP + Gemini architecture, breaks
               into atomic tasks
  ↓
(Cycle repeats through Tasks → Implement → Skill Design)
```

### What Makes Subagents "Intelligent"

Each subagent is designed with three components that activate reasoning (not just prediction):

**1. Persona** - Cognitive stance defining how to think
- Example: "You are a video production specialist who obsesses over marketing messaging and production quality before implementation"
- Not generic: "You are a helpful assistant"

**2. Questions** - Analytical framework guiding reasoning
- Example: "What makes this video compelling for SaaS marketing? What production constraints does Gemini.google.com have? How do we validate video quality?"
- Not vague: "Is this good?"

**3. Principles** - Decision criteria for evaluating options
- Example: "Every video must have clear call-to-action. Every specification must document Gemini constraints (max length, supported formats). Every test must validate marketing effectiveness."
- Not arbitrary: "Make it good"

### Example: Video Generation Specification Subagent

**Persona**: Video production specialist who thinks about marketing messaging and technical constraints before implementation

**Questions**:
- What's the core marketing message this video needs to communicate?
- What are the technical constraints of Gemini.google.com video generation (video length, format, quality)?
- What happens if Gemini API fails mid-recording? How do we retry?
- What's the simplest test that proves the video meets marketing objectives?

**Principles**:
- SMART criteria enforcement (Specific, Measurable, Achievable, Relevant, Testable)
- Every video specification documents target audience, marketing message, and success metrics
- Every API integration has documented failure cases and retry strategies
- Every generated video includes validation (is it > 80% of intended length? Does voiceover match script?)

**Result**: The Video Generation Specification Subagent doesn't just generate output - it **reasons through production problems** using video production expertise patterns.

### What Makes This "Reusable Intelligence"

- **Not video-generation-specific**: The Specification Subagent works for ANY feature needing clear requirements (database design, payment processing, file uploads)
- **Not rebuilt every project**: You don't re-teach "how to write specs" - the subagent embeds that expertise
- **Composable**: Lesson 9 teaches you to CREATE your own subagents (like `generate-video` and `upload-youtube`) using the P+Q+P pattern, adding them to your intelligence library

**Why "Vertical"**: Intelligence flows down a hierarchy (You → Orchestrator → Specialists), unlike Horizontal Intelligence which flows across time.

---

## Why This Matters for Video Generation

SaaS companies spend $5,000-$50,000 per professional demo video, yet struggle to produce them at scale. By building a system that **captures intelligence** (not just code), you're creating:

1. **Immediate value**: A video file generated with AI
2. **Reusable skills**: `generate-video` and `upload-youtube` that work across projects
3. **Compounding knowledge**: ADRs explaining Playwright + Gemini integration that future projects inherit
4. **Portfolio proof**: You've mastered SDD-RI by turning specification into production-quality deliverables

---

## Try With AI

Ready to understand how Spec-Kit Plus turns video generation work into reusable intelligence? Explore these patterns:

**Explore Intelligence Patterns:**
> "In Spec-Kit Plus, compare Horizontal Intelligence (ADRs/PHRs) vs Vertical Intelligence (Subagents). Imagine I build a video generation system in Project 1 and create 5 ADRs (about Playwright MCP, Gemini constraints, quality validation) and 12 PHRs (about what prompts generate compelling videos). How exactly does that intelligence help when I build an authentication system in Project 2? Show me concrete examples of what the AI agents would know."

**Practice P+Q+P for Video Production:**
> "Help me design a subagent for video script writing. Using Persona+Questions+Principles (P+Q+P), what persona should it adopt (think about marketing, production quality, SaaS context)? What questions should it ask to activate reasoning about video effectiveness? What principles should guide its decisions? Why does this approach work better than just asking 'write a video script'?"

**Calculate Intelligence Compounding:**
> "If Project 1 (video generation) creates 5 ADRs + 12 PHRs, Project 2 (auth system) creates 3 ADRs + 8 PHRs, and Project 3 (payment system) creates 2 ADRs + 6 PHRs, how much intelligence is available when I start Project 4? Explain why even projects with different domains benefit from accumulated intelligence."

**Apply to Your Domain:**
> "I work in [describe your domain/industry]. Help me design an intelligence architecture using Spec-Kit Plus for a project I'm planning. What Horizontal Intelligence should I capture (what architectural decisions repeat across projects)? What Vertical Intelligence should I create (what specialized subagents would accelerate future work)? Outline 3-5 intelligence artifacts I should build first."
