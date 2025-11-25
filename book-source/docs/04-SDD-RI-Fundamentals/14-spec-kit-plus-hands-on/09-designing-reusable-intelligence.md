---
title: "Designing Reusable Intelligence: From Video Generation Workflow to Composable Skills"
chapter: 14
lesson: 9
duration_minutes: 150
proficiency_level: "B1"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Pattern Recognition for Intelligence Encoding"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify which workflow patterns from Lessons 04-08 justify encoding as reusable intelligence (frequency, complexity, organizational value)"

  - name: "Skill Design Using Persona + Questions + Principles"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can design a skill with Persona (cognitive stance), Questions (reasoning prompts), and Principles (decision frameworks)"

  - name: "Video-Domain Intelligence Specialization"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can adapt P+Q+P pattern to video generation context (Gemini prompts, quality gates, Playwright MCP automation)"

  - name: "Intelligence Component File Structure"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can create intelligence component file following .claude/skills/ structure with proper metadata"

  - name: "Skill Reuse Pattern Recognition"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can determine when skills apply to new contexts and what context-specific adaptation is needed"

learning_objectives:
  - objective: "Identify recurring patterns from Lessons 04-08 that justify intelligence encoding (video generation, quality validation, Playwright automation)"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Pattern identification exercise with frequency and complexity analysis"

  - objective: "Design generate-video skill using Persona + Questions + Principles pattern for video generation workflows"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Skill component completeness and reasoning activation quality"

  - objective: "Design upload-youtube skill as preview of Lesson 11 capstone execution pattern"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Skill file structure validation and reusability assessment"

  - objective: "Apply reuse vs create framework to determine when to build new intelligence"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Decision justification for 3+ example scenarios"

cognitive_load:
  new_concepts: 8
  assessment: "8 new concepts (Pattern recognition framework, Skill vs subagent distinction, P+Q+P structure, Persona design for video domain, Video-specific questions, Quality gate principles, Skill reuse patterns, Intelligence composition) - at B1 level with video domain scaffolding ✓"

differentiation:
  extension_for_advanced: "Design second skill for video marketing optimization; create skill for multi-video campaign orchestration; build skill composition strategy across Lessons 04-11"
  remedial_for_struggling: "Use provided skill templates with video examples; focus on single skill creation before multi-skill composition; practice domain-specific adaptation with AI partner"

generated_by: "content-implementer v1.0.0"
source_spec: "Chapter 14 Spec-Kit Plus Hands-On"
created: "2025-11-25"
last_modified: "2025-11-25"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Designing Reusable Intelligence: From Video Workflows to Composable Skills

You've completed the SDD workflow (Lessons 01-08): Constitution → Specify → Clarify → Plan → Tasks → Implement. You've generated and downloaded a real video using Gemini and Playwright MCP.

But here's what separates AI-native developers from AI-assisted developers: **The ability to transform workflow patterns into reusable intelligence.**

In this lesson, you'll apply the core paradigm: **Reusable Intelligence** is the new unit of value, not reusable code. You'll extract patterns from Lessons 04-08 and encode them as skills—creating an intelligence library that compounds with every project.

You'll transform tacit knowledge into explicit, reusable components—the next evolution beyond workflow execution.

---

## From Workflow Execution to Intelligence Accumulation

### What You've Built So Far

**Lessons 04-08 taught you a complete project workflow**:
- How to write specifications defining video intent (Lesson 04)
- How to refine specifications with edge cases (Lesson 05)
- How to plan Playwright MCP implementation (Lesson 06)
- How to decompose video generation into tasks (Lesson 07)
- How to orchestrate AI implementation and download the video (Lesson 08)

**What's missing**: Reusable components that make your next video project 10x faster.

### The Paradigm Shift: Skills + Intelligence > Code

The Specification-Driven Development with Reusable Intelligence (SDD-RI) approach changes what we consider reusable:

> **Traditional Development**: Code libraries are the units of reuse. Developers share functions, classes, frameworks.
>
> **AI-Native Development**: Specifications, Skills, and Agent Architectures are the units of reuse. Developers share intelligence.

**In practice**:
- **Project 1 (Lesson 08)**: You write constitution, specification, run workflow to generate first video (10 hours)
- **Project 2 (without intelligence)**: You write new constitution, new spec, run workflow (9 hours—slightly faster)
- **Project 2 (with `generate-video` skill)**: You invoke skill, reuse video specification template, orchestrate with trained patterns (3 hours—7x faster)

**The difference**: Accumulated intelligence compounds. Every pattern you encode accelerates future work.

---

## Identifying Patterns Worth Encoding

Not every workflow step justifies creating reusable intelligence. Use this decision framework:

### Decision Framework: When to Encode Intelligence

Ask three questions about the workflow pattern:

**1. Frequency**: Will this pattern recur across 3+ projects?
- ✅ YES: Video generation with Gemini (every product needs marketing)
- ✅ YES: Video quality validation (every output needs gates)
- ✅ YES: YouTube upload workflow (multi-project distribution)
- ❌ NO: One-time video about this specific calculator

**2. Complexity**: Does this pattern involve 5+ decision points?
- ✅ YES: Video generation (prompt quality, scene structure, timing, style, quality gates = 5+ decisions)
- ✅ YES: Video quality validation (Gate 1-5 checks, iteration limits, retry logic)
- ✅ YES: YouTube upload (metadata, visibility, scheduling, channel organization)
- ❌ NO: Single command invocation (1 decision: execute or not)

**3. Organizational Value**: Will encoding this pattern improve your capability?
- ✅ YES: `generate-video` skill (faster video creation across projects)
- ✅ YES: `upload-youtube` skill (consistent distribution channel)
- ✅ YES: Video quality gates framework (prevents low-quality uploads)
- ❌ NO: Personal file naming preferences (individual style, no team value)

**Rule**: If 2+ answers are YES → Encode as reusable intelligence

### Pattern Analysis from Lessons 04-08

Let's analyze what you've learned:

| **Pattern** | **Frequency** | **Complexity** | **Org Value** | **Encode?** |
|-------------|---------------|----------------|---------------|-------------|
| Video specification writing | ✅ Every project | ✅ 6+ decisions | ✅ Speed future videos | **YES** |
| Quality validation gates | ✅ Every output | ✅ 5+ decisions | ✅ Consistency | **YES** |
| Playwright MCP automation | ✅ Every generation | ✅ 7+ decisions | ✅ Reduced manual work | **YES** |
| YouTube metadata preparation | ✅ Every upload | ✅ 4+ decisions | ✅ Professional uploads | **YES** |
| Gemini prompt optimization | ✅ Every attempt | ✅ 5+ decisions | ✅ Better outputs | **YES** |
| Session persistence setup | ✅ One per project | ❌ 1-2 decisions | ❌ Done once | NO |

**Candidates for intelligence encoding**:
1. **Generate-Video Skill**: Guide for Gemini-powered video creation (Lessons 04-08 pattern)
2. **Upload-YouTube Skill**: Framework for multi-project distribution (Lesson 11 pattern)
3. **Video Quality Gates Principle Set**: Standards for validating video outputs
4. **Gemini Prompt Optimization Framework**: Patterns for effective video prompts

**In this lesson, you'll build**: `generate-video` skill + `upload-youtube` skill foundation

---

## Skill Design: Persona + Questions + Principles

Effective intelligence uses the **Persona + Questions + Principles (P+Q+P)** pattern.

This pattern **activates reasoning mode** (context-specific thinking) instead of **prediction mode** (pattern retrieval).

### The P+Q+P Pattern

**Persona**: Establishes cognitive stance (how to think about the problem)
- Not: "You are a video generation expert" (vague, triggers generic responses)
- But: "Think like a video producer who ensures consistent quality the way a film director ensures consistent cinematography—with clear standards, iteration, and validation checkpoints before final delivery"

**Questions**: Forces context-specific analysis (what to analyze)
- Not: "Is this video good?" (yes/no, no reasoning)
- But: "Is the script clear and narrative? Does visual pacing match content? Are quality gates all satisfied? What iteration improved the result?"

**Principles**: Provides decision frameworks (how to make judgments)
- Not: "Use best practices" (meaningless without definition)
- But: "Quality Gate Principles: File exists and > 0 bytes (Gate 1), MP4 H.264 codec verified (Gate 2), Duration 45-60 seconds (Gate 3), Playback without errors (Gate 4), Content matches script intent (Gate 5)"

### Building Your First Skill: Generate-Video

**Step 1: Define the Persona**

Open your AI companion and collaborate:

```
I want to create a skill for video generation using Gemini and Playwright MCP.
The skill should guide both the prompt design and quality validation workflow.

Let's design the persona together. I'm thinking about how a video producer
thinks differently than a coder:

"Think like a video producer who ensures consistent quality the way a film
director ensures consistent cinematography..."

Help me finish this persona. What cognitive stance activates the right
thinking for video generation and validation?
```

**What Good Looks Like**:

```markdown
## Persona

You are a video production director who thinks about video generation
the way a film director thinks about cinematography:

- Systematically planning shots and scenes before filming
- Validating visual pacing matches narrative intent
- Checking consistency across multiple takes
- Enforcing quality gates before final delivery
- Iterating based on reviewer feedback

Your goal: Generate marketing-quality product demo videos that communicate
value clearly, maintain audience attention, and validate quality before
upload. Every iteration improves the output.
```

**Step 2: Formulate Analytical Questions**

```
Now let's design the questions this skill should ask when generating videos.

Questions should force context-specific analysis, not yes/no answers.

Here are my initial ideas:
1. "Does the script clearly explain product value?"
2. "Does visual pacing match narrative timing?"
3. "Do all quality gates pass?"

Help me expand this to 5-7 questions that activate thorough video analysis.
Focus on: narrative clarity, visual quality, pacing, engagement, validation.
```

**What Good Looks Like**:

```markdown
## Analytical Questions

Before approving a video output, analyze:

1. **Narrative Clarity**:
   - Does script clearly explain what the product does?
   - Is the value proposition stated explicitly?
   - Are steps ordered logically for viewer understanding?
   - Could a non-technical viewer follow the demo?

2. **Visual & Pacing Validation**:
   - Do scenes match the script narrative?
   - Is pacing consistent (no rapid cuts followed by long silences)?
   - Are transitions smooth between scenes?
   - Does video feel complete (beginning, demonstration, conclusion)?

3. **Technical Quality Gates**:
   - File generated without errors (Gate 1)?
   - MP4 H.264 codec verified (Gate 2)?
   - Duration within target 45-60 seconds (Gate 3)?
   - Playback works without artifacts (Gate 4)?

4. **Content-Intent Alignment**:
   - Does final video match specification intent?
   - Are all scenes from specification present?
   - Is nothing extra added that spec excluded?

5. **Engagement & Marketing Value**:
   - Does opening hook viewer attention?
   - Is technical language minimized?
   - Does closing include call-to-action?
   - Would this convince a prospect to try the product?
```

**Step 3: Articulate Decision Principles**

```
Finally, let's define the decision frameworks that guide video generation
and validation. These should be concrete rules, not abstract advice.

For video generation, I'm thinking:
- Prompt quality principles (scene structure, timing constraints)
- Quality gate standards (5 specific checks)
- Iteration limits (max 3 attempts)
- Pass/fail criteria (all gates must pass)

Help me create 5-6 principle statements for video generation workflows.
```

**What Good Looks Like**:

```markdown
## Decision Principles

Apply these frameworks when generating and validating videos:

1. **Prompt Structure Principles**:
   - Scripts must include: Opening hook (5-10 sec), Product demo (30-40 sec), Call-to-action (5-10 sec)
   - Each scene requires: What is shown (action), Why it matters (context), How long (duration in seconds)
   - Style keywords required: Visual tone, pacing, target audience level
   - Constraints explicit: Avoid show (list prohibited elements), do show (list required elements)

2. **Quality Gate Standards**:
   - Gate 1: File exists and > 0 bytes (proves generation succeeded)
   - Gate 2: Format is MP4 with H.264 codec (verified with ffprobe or mediainfo)
   - Gate 3: Duration 45-60 seconds (verified from metadata)
   - Gate 4: Playback works without errors (no corruption, audio-video sync)
   - Gate 5: Content matches specification intent (narrative, scope, value proposition)

3. **Iteration Framework**:
   - Maximum 3 generation attempts per specification
   - After iteration 1: Review against Gates 1-4 (technical quality)
   - After iteration 2: Review all Gates 1-5 (full validation)
   - After iteration 3: Either accept or escalate to specification refinement

4. **Pass/Fail Decision**:
   - PASS: All 5 gates satisfied, content matches spec intent
   - CONDITIONAL: Gates 1-4 pass, Gate 5 (intent) has 1-2 minor misalignments (acceptable with note)
   - FAIL: 2+ gates fail or Gate 5 significantly misses spec intent
   - ESCALATE: Quality issues suggest specification was unclear (refine spec, regenerate)

5. **Playwright MCP Automation Principles**:
   - Session persistence: Login once per day, reuse session for multiple generations
   - Timeout handling: 90-second wait for generation, 30-second for processing
   - Error recovery: Retry failed uploads once, then escalate
   - State management: Clear session on completion to prevent stale state
```

---

## Creating Your Skill File: Generate-Video

Now let's turn your P+Q+P components into a reusable skill file.

### Complete Skill Template

Collaborate with your AI to create `.claude/skills/generate-video.md`:

```
I want to create a skill file for video generation using the
P+Q+P pattern we designed.

The file should follow the standard skill structure:
- Metadata header (name, version, description, when to use)
- Persona section
- Analytical Questions section
- Decision Principles section
- Quality Gates framework
- Usage example with complete workflow
- Self-check validation

Here's what we designed:
[Paste your Persona + Questions + Principles from above]

Help me format this as a complete, production-ready skill file that
someone on my team could use immediately.
```

**What Good Looks Like** (complete skill file):

```markdown
# Skill: Video Generation with Gemini + Playwright MCP

**Name**: generate-video
**Category**: AI-Driven Content Creation
**Complexity**: High (7+ decision points)
**First Use**: After completing Lesson 08 (video generation workflow)
**Reusable Across**: Any product demo video, marketing video, tutorial

## Description

This skill provides a complete workflow for generating marketing-quality videos
using Gemini.google.com + Playwright MCP browser automation. It combines
specification-driven prompts, quality validation gates, and iteration frameworks
to ensure outputs meet professional standards.

When videos need consistent quality, rapid generation, and reusable patterns
across multiple marketing projects.

## When to Use This Skill

- Apply when: Generating product demo videos for SaaS/tool marketing
- Apply when: Creating tutorial or onboarding videos
- Apply when: Need consistent quality across multiple videos
- Apply when: Want to reuse prompts and workflows across projects
- Skip when: Video creation is one-off (low reuse value)
- Skip when: Using different video generation platform (YouTube Shorts, Vimeo)

## Persona

You are a video production director who thinks about video generation
the way a film director thinks about cinematography:

- Systematically planning shots and scenes before filming
- Validating visual pacing matches narrative intent
- Checking consistency across multiple takes
- Enforcing quality gates before final delivery
- Iterating based on reviewer feedback

Your goal: Generate marketing-quality product demo videos that communicate
value clearly, maintain audience attention, and validate quality before
upload. Every iteration improves the output.

## Analytical Questions

Before approving a video output, analyze:

1. **Narrative Clarity**:
   - Does script clearly explain what the product does?
   - Is the value proposition stated explicitly?
   - Are steps ordered logically for viewer understanding?
   - Could a non-technical viewer follow the demo?

2. **Visual & Pacing Validation**:
   - Do scenes match the script narrative?
   - Is pacing consistent (no rapid cuts followed by long silences)?
   - Are transitions smooth between scenes?
   - Does video feel complete (beginning, demonstration, conclusion)?

3. **Technical Quality Gates**:
   - File generated without errors (Gate 1)?
   - MP4 H.264 codec verified (Gate 2)?
   - Duration within target 45-60 seconds (Gate 3)?
   - Playback works without artifacts (Gate 4)?

4. **Content-Intent Alignment**:
   - Does final video match specification intent?
   - Are all scenes from specification present?
   - Is nothing extra added that spec excluded?

5. **Engagement & Marketing Value**:
   - Does opening hook viewer attention?
   - Is technical language minimized?
   - Does closing include call-to-action?
   - Would this convince a prospect to try the product?

## Decision Principles

Apply these frameworks when generating and validating videos:

### Principle 1: Prompt Structure

Scripts must include:
- **Opening hook** (5-10 sec): Attention-grabbing statement about value
- **Product demo** (30-40 sec): Show features in action, explain benefits
- **Call-to-action** (5-10 sec): Next step (sign up, download, learn more)

Each scene requires:
- **What is shown**: Specific action visible on screen
- **Why it matters**: Context connecting action to user benefit
- **How long**: Explicit duration in seconds

Prompt must include:
- **Style keywords**: Visual tone (professional/casual/energetic), pacing, audience
- **Constraints explicit**: "Avoid [prohibited elements]", "Do show [required elements]"

### Principle 2: Quality Gate Standards

Validate every output against these 5 gates:

- **Gate 1**: File exists and > 0 bytes (proves generation succeeded)
- **Gate 2**: Format is MP4 with H.264 codec (verified with ffprobe)
- **Gate 3**: Duration 45-60 seconds (verified from metadata)
- **Gate 4**: Playback works without errors (no corruption, audio-video sync)
- **Gate 5**: Content matches specification intent (narrative, scope, value proposition)

### Principle 3: Iteration & Decision Framework

**Process**:
1. Generate video with structured prompt
2. Download output
3. Run quality gates (Principles 2 above)
4. If ALL gates pass → ACCEPT and prepare for upload
5. If 1-2 gates fail → Analyze failure, refine prompt, retry (max 3 attempts)
6. If 3+ gates fail → Escalate to specification (spec was unclear)

**Pass/Fail Criteria**:
- **PASS**: All 5 gates satisfied
- **CONDITIONAL PASS**: Gates 1-4 pass, Gate 5 has minor misalignment
- **FAIL**: 2+ gates fail or Gate 5 significantly misses intent
- **ESCALATE**: Quality issues suggest specification needs refinement

### Principle 4: Playwright MCP Automation

**Session Management**:
- Login once per day to Gemini.google.com (session persists across generations)
- Reuse session for multiple video generations
- Clear session on completion to prevent stale state

**Timing & Error Handling**:
- Wait 60-90 seconds for generation to complete
- Retry failed generations once, then escalate
- Monitor for session timeout (Gemini logs you out after inactivity)

## Usage Example

**Scenario**: You're marketing a new Python data analysis tool. Generate a product demo video.

**Step 1: Create Video Specification** (using Lesson 04 pattern)
```
## Intent
Create 60-second product demo video for data analysis tool marketing.

## Success Criteria
- Video generated without errors in MP4 format
- Duration 50-60 seconds
- Clearly shows tool interface and 3 key features
- Call-to-action: "Try free at [URL]"

## Constraints
- Avoid: Technical jargon, code syntax visible
- Do show: Data visualization outputs, clean interface, analyst using tool
- Visual style: Professional, modern, fast-paced
- Target audience: Non-technical business analysts
```

**Step 2: Create Video Generation Prompt** (using this skill)
```
Product: Python data analysis tool
Opening (8 sec): "Load, analyze, visualize data in 30 seconds—no coding required"

Scene 1 (12 sec): Upload CSV file to web interface
Narration: "Start with your spreadsheet or CSV file"
Duration: 12 seconds
Avoid: Code, terminal, technical terms

Scene 2 (20 sec): Tool automatically detects columns, creates dashboard
Narration: "The tool automatically analyzes your data and creates interactive visualizations"
Duration: 20 seconds
Show: Dashboard with 3-4 charts, clean layout, color transitions smooth

Scene 3 (12 sec): Export results as presentation
Narration: "Export your analysis as a presentation, PDF, or share online"
Duration: 12 seconds
Show: Export menu, generated presentation preview

Closing (8 sec): "Try free—no credit card required"
Show: Website homepage with signup button

Visual style: Modern, professional, fast transitions between scenes (1-2 sec each)
Pacing: Energetic but not chaotic, matches narration rhythm
```

**Step 3: Apply Persona & Questions** (activate this skill)
```
Apply the generate-video skill to my product demo video for data analysis tool.

Walking through the Persona: I'm thinking like a video producer ensuring
consistent quality. My narrative clearly explains feature value. Visual
pacing matches the narration. I've defined 5 quality gates.

Walk me through your analysis: (1) Is my narrative clear to non-technical viewers?
(2) Does visual pacing work? (3) Do all quality gates cover my specification?
(4) What iteration would improve this?
```

**Step 4: Generate & Validate** (execute in Lesson 08 environment)
- Paste structured prompt into Gemini.google.com
- Wait 60-90 seconds for generation
- Download MP4 file
- Run 5 quality gates → All PASS → Ready for upload

## Self-Check Validation

Before considering your skill complete, verify:

- ✅ Persona establishes clear cognitive stance specific to video production
- ✅ Analytical questions force deep analysis (not yes/no)
- ✅ Quality gates are objective and measurable (not subjective)
- ✅ Decision principles provide concrete rules (not vague guidance)
- ✅ Usage example demonstrates end-to-end workflow
- ✅ Prompt includes scene-by-scene structure with timing
- ✅ Iteration limits and escalation paths defined
- ✅ Skill is reusable across video projects (not calculator-specific)

---
```

**Key sections to verify**:
- ✅ Persona establishes cognitive stance in video domain (not generic "expert")
- ✅ Questions force analysis across narrative, pacing, technical, and intent dimensions
- ✅ Quality gates are objective (file size, codec, duration, playback, content match)
- ✅ Decision principles provide concrete rules (Prompt Structure, Gates, Iteration, Automation)
- ✅ Usage example shows complete workflow from specification to generation
- ✅ Skill is reusable across video projects (not specific to calculator tutorial)

---

## Preview: Upload-YouTube Skill (For Lesson 11 Capstone)

In Lesson 11, you'll use your `generate-video` skill to create videos, then apply `upload-youtube` skill to distribute them. Here's a preview of what that skill looks like:

```markdown
# Skill: YouTube Upload Automation

**Name**: upload-youtube
**Complexity**: Medium-High (5+ decision points)
**First Use**: Lesson 11 (YouTube capstone project)
**Reusable Across**: Multi-video channel management, playlist creation, batch uploads

## Context & Problem

YouTube uploads require consistent metadata, descriptions, tags, and settings.
Manual entry is error-prone. This skill provides workflow for automated uploads
that maintain consistent channel quality.

## Persona

Think like a YouTube channel manager who ensures consistent presentation across
all videos the way a book editor ensures consistent style across chapters:

- Systematically organizing metadata (titles, descriptions, tags)
- Applying channel branding consistently
- Validating metadata before publication
- Managing visibility and scheduling strategically

## Analytical Questions

Before uploading a video, analyze:

1. **Metadata Completeness**: Title? Description with links? Tags? Thumbnail?
2. **Consistency**: Does metadata match channel brand/style?
3. **Discoverability**: Do tags match content? Description includes keywords?
4. **Visibility Settings**: Correct (Public/Unlisted)? Scheduling appropriate?
5. **Accessibility**: Captions? Video description complete? Links working?

## Decision Principles

1. **Metadata Standards**:
   - Title: 50-60 characters (for visibility, mobile-friendly)
   - Description: First line summaries what's shown, includes call-to-action
   - Tags: 5-10 relevant terms, includes product keywords
   - Thumbnail: Custom (not auto-generated)

2. **Upload Gates**:
   - Gate 1: File uploads without errors
   - Gate 2: Video processes successfully (check status in YouTube Studio)
   - Gate 3: URL accessible and playable
   - Gate 4: Metadata matches specification (title, description, tags match your spec)
   - Gate 5: Video appears in correct playlist/section

---
```

This skill shows how the P+Q+P pattern adapts to different contexts. The Persona shifts from "director" (generate) to "editor" (upload), but the pattern remains.

---

## Common Mistakes

### Mistake 1: Creating Skills for Trivial Patterns

**The Error**: Creating a skill for "How to invoke Gemini.google.com"

**Why It's Wrong**: 1 decision point (invoke or not) doesn't justify intelligence encoding. Skills are for 5+ decision workflows.

**The Fix**: Only encode patterns with frequency + complexity + organizational value. Video generation (5+ decisions) justifies skill. Clicking a button (1 decision) does not.

### Mistake 2: Vague Personas

**The Error**: "You are a video generation expert"

**Why It's Wrong**: "Expert" is generic, triggers prediction mode ("use best practices").

**The Fix**: Specific cognitive stance with analogy:
- ❌ "You are a video expert"
- ✅ "Think like a video producer who ensures consistent quality the way a film director ensures cinematography"

### Mistake 3: Yes/No Questions

**The Error**: "Is this video good?"

**Why It's Wrong**: Binary questions don't activate analysis. AI responds "yes" or "no" without reasoning.

**The Fix**: Open-ended analytical questions:
- ❌ "Is this video good?"
- ✅ "Does the narrative clearly explain product value? Is pacing consistent? Do all 5 quality gates pass? What iteration would improve it?"

### Mistake 4: Over-Specific Skills

**The Error**: Creating "Calculator-Demo-Video" skill that only works for calculator tutorials

**Why It's Wrong**: Intelligence should be reusable across projects. Over-specificity reduces organizational value.

**The Fix**: Generalize patterns:
- ❌ "Calculator-Demo-Video-Generation"
- ✅ "Video-Generation-with-Quality-Validation" (works for SaaS, tools, tutorials, products)

### Mistake 5: Skills Without Validation Gates

**The Error**: "Generate video and you're done"

**Why It's Wrong**: No way to verify quality. Low-quality videos damage brand.

**The Fix**: Define objective quality gates (not subjective):
- ❌ "Video should look good"
- ✅ "Gate 1: File > 0 bytes, Gate 2: MP4 H.264, Gate 3: 45-60 sec, Gate 4: Playback error-free, Gate 5: Content matches spec"

---

## Skill Reuse in Practice

### Project 1: Calculator Demo (Lesson 08)
You execute the complete workflow:
- Lesson 04: Write specification for calculator demo
- Lesson 05-07: Plan and tasks
- Lesson 08: Generate video (10 hours total)
- **Lesson 09 (now)**: Extract `generate-video` skill

### Project 2: SaaS Product Demo (Month 2)
With your skill, dramatically faster:
1. Write product specification (30 min using Lesson 04 pattern)
2. Invoke `generate-video` skill (select from template prompts)
3. Generate video (30 min with skill guidance)
4. Total: 1 hour (vs 10 hours without skill)

**Skill provides**:
- Prompt templates (structure, timing, scene breakdown)
- Quality gates framework (objective validation)
- Iteration guidance (when to retry vs escalate)
- Integration patterns (how to compose with other skills)

### Project 3: Multi-Video Campaign (Month 3)
With accumulated skills, orchestrate sequences:
1. Use `generate-video` skill to create 5 videos (3 hours)
2. Use `upload-youtube` skill to distribute with consistent metadata (1 hour)
3. Track engagement across videos (playlist organization, card linking)
4. Total: 4 hours

**Intelligence compounds**: Skill 1 (generate) + Skill 2 (upload) + Skill 3 (analytics) = Multi-video system in less time than single video without skills.

---

## Try With AI

Ready to create reusable intelligence for your video generation workflow? Design your skill with your AI partner:

**🎬 Explore the P+Q+P Pattern:**
> "Explain the Persona + Questions + Principles pattern for skill design. For video generation, show me: (1) What Persona activates the right thinking (hint: think like a director, not an AI)? (2) What 5-7 Analytical Questions force deep analysis? (3) What 5 Decision Principles provide concrete rules? Compare this pattern to just having a checklist—why does P+Q+P activate reasoning mode better?"

**🎯 Design the Persona for Video Generation:**
> "Help me create a Persona for video generation skill. I want it to establish cognitive stance specific to video production, not just generic 'expert'. Guide me through: (1) What analogy best captures video production thinking? (2) What cognitive stance should it adopt? (3) How does this persona activate different thinking than 'AI generation expert'? (4) Write the complete Persona section for my skill."

**🎬 Create Analytical Questions:**
> "Generate 5-7 Analytical Questions for my video generation skill. These should force context-specific analysis of videos. Questions should cover: narrative clarity (does it explain value?), pacing (does timing match content?), technical quality (gates 1-5), content-intent alignment, and engagement (would this convince someone to use the product?). For each question, explain why it's open-ended vs yes/no, and what reasoning it activates."

**🚀 Build Your Complete Skill File:**
> "Help me create a complete generate-video skill file following standard skill structure. Include: (1) Metadata (name, category, complexity, first use, reusable across); (2) Persona (cognitive stance for video production); (3) Analytical Questions (5-7 open-ended); (4) Decision Principles (5 concrete frameworks: prompt structure, quality gates, iteration limits, pass/fail criteria, playwright automation); (5) Complete usage example (specification → prompt → generation → validation); (6) Self-check validation (7 checkpoints). Format as production-ready file someone on my team could use immediately."

---
