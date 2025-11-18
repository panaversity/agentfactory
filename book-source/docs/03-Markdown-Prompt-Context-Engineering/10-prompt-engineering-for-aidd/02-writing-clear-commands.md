# Lesson 2: Writing Clear Commands — Specification-First Fundamentals

---

## What Should This Lesson Enable You to Do?

By the end of this lesson, you should be able to:

- **Write specifications that define WHAT you want before attempting HOW to ask for it**
- **Distinguish between vague and falsifiable specifications** and recognize why clarity compounds output quality
- **Apply a specification-first framework** to prompt design (outcome → success criteria → constraints → prompt)
- **Recognize the relationship between specification quality and AI output quality**
- **Evaluate prompt specifications for completeness** using a decision framework

**Success criteria**: You can take a vague problem statement ("evaluate this codebase") and transform it into a clear specification that guides accurate AI reasoning.

---

## The Professional Reality: Vague Specifications Fail

### The Scenario

You are a **Technical Writer** creating documentation for your team's new Python library. Your manager asks:

> "We need docs for our authentication library. Can you write something that explains how to use it?"

This is your raw requirement. **Stop and think**: What does "explains how to use it" mean? Installation guide? API reference? Tutorial with examples? Security best practices?

**Without a specification**, you ask an AI agent:
```
"Write documentation for this authentication library"
```

**The AI responds** with generic README structure (Installation, Usage, License). You get 60% of what you need.

**With a specification**, you first define:
```
Intent: Create user-facing documentation for developers integrating our auth library
Success criteria: Quickstart guide (5 min setup), authentication flow diagram, 3 code examples (basic/OAuth/2FA), troubleshooting section
Constraints: Audience is intermediate Python developers; library uses FastAPI; docs must be Markdown for GitHub
Risks to validate: Common integration errors, security misconfigurations
```

**Now you ask the AI** with that specification as context. You get 95% of what you need.

**The difference is massive**: Same AI, same library, but specification quality determines output quality.

---

## Concept 1: Specification vs. Vague Request

### What's the Difference?

**Vague Request** (60% quality):
- Single sentence or question
- No success criteria defined
- No constraints mentioned
- Ambiguous terms ("better", "understand", "analyze")
- AI guesses what you want

Example:
```
"Write documentation for this library"
```
→ AI responds with generic README template, not user-focused guide

**Specification** (95% quality):
- Intent clearly stated (what problem are you solving?)
- Success criteria defined (what does a good answer look like?)
- Constraints listed (what limits exist?)
- Non-goals defined (what's explicitly out of scope?)
- Falsifiable (AI can verify if it answered correctly)

Example:
```
Intent: Create onboarding documentation for developers new to our markdown parser library
Success criteria: Quickstart (working example in 5 min), 3 usage scenarios (basic/tables/extensions), API reference table, troubleshooting section
Constraints: Markdown format, audience is Python developers with basic markdown knowledge, must include code examples with expected outputs
Non-goals: Markdown syntax tutorial, performance optimization guide, comparison to other parsers
```
→ AI responds with exactly what you specified

### Socratic Discovery

**Question 1**: If I ask an AI agent "Write documentation for this library" without any context, what could the agent produce? (What would 60% of the possible output look like?)

**Question 2**: What would 95% look like? What information would make you confident the documentation serves your users?

**Key Insight**: The gap between 60% and 95% is **specification clarity**. Not smarter AI, not longer prompts—specification quality.

---

## Concept 2: Falsifiable Success Criteria

### What Makes a Specification Falsifiable?

**Falsifiable** means: "I can verify if you answered my question correctly."

**Non-falsifiable specification**:
```
Success criteria: "Write good documentation"
```
→ Vague. How do you verify "good"? AI could respond with anything.

**Falsifiable specification**:
```
Success criteria:
1. Provide quickstart guide (user can run example in 5 minutes)
2. Include 3 code examples (basic usage, advanced feature, error handling)
3. Add troubleshooting section (5 most common errors with solutions)
4. Answer: "Can a new developer integrate this library in < 30 minutes?"
```
→ Clear. You can verify each criterion independently.

### Exercise: Falsifiability Assessment

For each specification below, identify which are falsifiable:

**Specification A**:
```
"Write documentation that's easy to understand"
```
→ Is this falsifiable? Why or why not?
(Answer: No. "Easy to understand" is subjective. For whom? What's their background?)

**Specification B**:
```
"Create documentation with: quickstart guide (working example < 5 min), API reference table (all public functions), 3 code examples (with expected outputs), troubleshooting section (5 common errors)"
```
→ Is this falsifiable? Why?
(Answer: Yes. You can verify each element independently: Does quickstart exist? Does it have working example? Are all functions documented?)

**Specification C**:
```
"Generate README with: installation instructions (pip install), usage section (import statement + basic example), configuration options (environment variables table), license section (MIT)"
```
→ Is this falsifiable? Why?
(Answer: Yes. Each section has clear presence/absence criteria. You can verify: Are installation steps present? Is there a usage example?)

---

## Concept 3: The Specification Layers Framework

### Four Layers of Specification Detail

When you write a specification, include these layers (in order):

**Layer 1: Intent** (Why are you asking this question?)
```
"We're creating developer documentation for our new markdown parser library"
```

**Layer 2: Success Criteria** (What does a good answer look like?)
```
"Need: quickstart guide (< 5 min setup), usage examples (3 scenarios), API reference, troubleshooting section"
```

**Layer 3: Constraints** (What limits exist?)
```
"Audience: Python developers (intermediate level)
Format: Markdown for GitHub
Style: Match existing docs (code-first examples, minimal prose)"
```

**Layer 4: Prompt** (Now ask the question)
```
"Generate documentation for markdown parser [with Layer 1-3 context provided]"
```

### Why This Order Matters

**If you skip Layer 1-3 and jump to Layer 4**: AI guesses what you want

**If you include Layer 1-3**: AI has framework for answering precisely

**Example comparison**:

**Without layers**:
```
User: "Write documentation for this library"
AI: Generic README with standard sections (Installation, Usage, License)
Result: 60% relevance
```

**With layers**:
```
Layer 1: "We're creating docs for developer onboarding"
Layer 2: "We need: quickstart (< 5 min), 3 examples, API reference, troubleshooting"
Layer 3: "Audience: Python devs (intermediate); Format: Markdown; Style: code-first examples"
User: "Generate documentation for markdown parser"
AI: Targeted documentation with quickstart, code-heavy examples, troubleshooting for common errors
Result: 95% relevance
```

---

## Concept 4: Contrast Exercise — Specification vs. Non-Specification

### Before and After: Real Examples

**Example 1: Library Documentation**

❌ **Vague** (without specification):
```
"Write documentation for this library"
```

✅ **Specified** (with layers):
```
Intent: Create onboarding docs for developers new to our HTTP client library
Success Criteria: Quickstart guide (< 5 min), 3 code examples (GET/POST/auth), API reference table, error handling section
Constraints: Audience is Python developers (intermediate); Markdown format; Must work with code version 2.0+
Non-Goals: HTTP protocol tutorial, performance benchmarks, comparison to requests/httpx
```

**The difference**: First gets generic README. Second gets user-focused onboarding guide.

---

**Example 2: Tutorial Content**

❌ **Vague**:
```
"Create a tutorial for beginners"
```

✅ **Specified**:
```
Intent: Teach markdown basics to content creators (non-developers) for documentation workflows
Success Criteria: 10-minute walkthrough (headings/lists/links/code blocks), 5 practice exercises (progressive difficulty), cheat sheet reference
Constraints: Audience has zero programming experience; Must use visual examples; Practice exercises use real-world scenarios (blog posts, documentation)
Non-Goals: HTML/CSS integration, advanced markdown extensions, static site generators
```

---

## Concept 5: Building Your Specification

### The Specification Template

Use this structure every time you need AI to create documentation or content:

```
## Intent (1-2 sentences)
Why are you creating this content? What purpose does it serve?

## Success Criteria (3-5 items)
What would complete, useful documentation include?

## Constraints (3-5 items)
What limits exist? (audience, format, length, style?)

## Non-Goals (2-3 items)
What are you explicitly NOT including?

## Context Layers (optional)
What background does the AI need to understand?
- Content purpose
- Audience background
- Existing documentation style
- Integration requirements
```

### Exercise: Write Your Specification

**Scenario**: You need to create a README for your team's new CLI tool that helps developers manage environment variables.

**Try this**: Write a 4-layer specification for this documentation (Intent, Success Criteria, Constraints, Non-Goals).

**Hint**: What would make this README useful for new users? What constraints apply? What's out of scope?

(Example answer at end of lesson)

---

## Synthesis: From Vague to Clear

### The Journey

1. **Start**: Vague question ("Write documentation")
2. **Layer 1**: Add Intent ("We're creating onboarding docs for new developers")
3. **Layer 2**: Add Success Criteria ("We need quickstart + 3 examples + API reference + troubleshooting")
4. **Layer 3**: Add Constraints ("Audience: Python devs (intermediate), Markdown format, code-first style")
5. **Result**: Clear specification that guides accurate AI content creation

### Why This Matters Strategically

**Strategic content creation is built on clear specifications, not smart tools.**

Before AI agents, you'd hire a technical writer to create documentation. You wouldn't just say "write docs." You'd provide a specification:
- "What we're creating it for" (Intent)
- "What the documentation needs to cover" (Success Criteria)
- "What we already know about the audience" (Constraints)
- "What's out of scope" (Non-Goals)

**AI agents have inverted this**: We think more capability means we need less clarity. Actually, it's the opposite. More powerful tools need clearer specifications.

---

## Self-Assessment: Specification Evaluation Exercise

### Exercise 1: Identify the Specification Gaps

Below is a specification someone wrote. Identify what's missing or unclear:

```
Intent: Write good documentation
Success Criteria: Make it easy to understand
Constraints: I know markdown
Non-Goals: I don't want to explain everything
```

**What's missing?**
- Intent is too vague ("good documentation" is subjective)
- Success Criteria is not falsifiable ("easy to understand" can't be verified)
- Constraints incomplete (what's the audience? what format? what content type?)

**How would you improve it?**

---

### Exercise 2: Refactor a Vague Request

**Original request**:
```
"Write a tutorial for this tool"
```

**Your task**: Refactor this into a specification with Intent, Success Criteria, Constraints, Non-Goals

**Possible refactoring**:
```
Intent: Create beginner tutorial for our CLI tool (environment variable manager)
Success Criteria: Installation guide (< 2 min), 3 usage examples (set/get/delete vars), troubleshooting section (5 common errors)
Constraints: Audience is developers new to CLI tools, Markdown format, must include copy-paste commands
Non-Goals: Advanced configuration options, shell scripting tutorial, comparison to other env managers
```

---

## What's Next

You now understand **specification as the foundation** of clear AI prompting. In Lesson 3, you'll apply this framework in collaboration with an AI agent, using what you've learned here to guide systematic codebase analysis.

---

## Try With AI

**This is a Stage 1 lesson (manual foundation)**: Use this reflection exercise without AI tools.

### Reflection Exercise: Specification-First Thinking

Answer these questions in writing (no AI assistance):

1. **Recall**: Think of a time you asked someone (a colleague, teacher, AI) a vague question and got an unhelpful answer. What was missing in your question?

2. **Apply**: Take that scenario and write a proper specification (Intent, Success Criteria, Constraints, Non-Goals) that would have gotten a better answer.

3. **Synthesize**: How is writing a clear specification like giving someone detailed instructions before asking them to do a task? What's the parallel?

4. **Evaluate**: For your own work (evaluating codebases, learning frameworks), where could you benefit from writing specifications first instead of asking vague questions?

**After completing this reflection, you're ready for Lesson 3, where you'll practice this with an actual AI agent.**

---

## Lesson Metadata

- **Stage**: 1 (Manual Foundation — NO AI tools)
- **Modality**: Specification-first + Socratic dialogue
- **Concepts**: 5 (Vague vs. falsifiable specs, 4-layer framework, specification quality, intent/criteria/constraints, falsifiability)
- **Cognitive Load**: 5 ≤ 7 (B1 tier Stage 1 limit ✅)
- **AI Tools Used**: NONE (Stage 1 principle)
- **Duration**: 50-60 minutes
- **Version**: 1.0.0
- **Constitution**: v6.0.0 Compliance
- **Generated**: 2025-01-18
- **Feature**: 025-chapter-10-redesign
- **Source Spec**: `specs/025-chapter-10-redesign/plan.md` (Lesson 2 structure)

---

## Answer Key: Specification Exercise (Concept 5)

**Example specification for "Create README for CLI tool managing environment variables"**:

```
Intent: Create onboarding documentation for developers new to our env-manager CLI tool

Success Criteria:
1. Installation guide (pip install + verify setup < 2 min)
2. Quickstart example (set, get, delete env vars with copy-paste commands)
3. Three usage scenarios (local dev, testing, CI/CD integration)
4. Troubleshooting section (5 common errors: permission issues, path not found, syntax errors, conflicts, shell compatibility)
5. API reference table (all commands with arguments and examples)

Constraints:
- Audience: Developers familiar with command line but new to env management tools
- Format: Markdown for GitHub
- Style: Code-first examples (minimal prose, max clarity)
- Length: Must fit in GitHub's README preview (< 5-minute read)

Non-Goals:
- Shell scripting tutorial
- Environment variable concepts explained from scratch
- Comparison to other env managers (direnv, dotenv)
- Advanced configuration (encryption, remote storage)
```

This specification guides AI to give you exactly what you need for user onboarding.
