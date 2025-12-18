---
title: "Compiling MCP to Skills"
sidebar_position: 9
chapter: 5
lesson: 9
duration_minutes: 25

# PEDAGOGICAL LAYER METADATA
primary_layer: "Layer 2"
layer_progression: "L2 (AI Collaboration) - applying skills + MCP knowledge from previous lessons"
layer_1_foundation: "Completed in Lessons 04-06 (Skills) and 08 (MCP)"
layer_2_collaboration: "Introspecting MCP servers with Claude, compiling to skills, validating token reduction"
layer_3_intelligence: "N/A"
layer_4_capstone: "N/A"

# HIDDEN SKILLS METADATA
skills:
  - name: "MCP-to-Skill Compilation"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can identify MCP token bloat, introspect server definitions, and compile to skill + script format"

learning_objectives:
  - objective: "Identify when MCP servers cause token bloat"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Recognize high-token MCP patterns and articulate the problem"
  - objective: "Introspect an MCP server's tool definitions using Claude"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successfully extract tool definitions from a running MCP server"
  - objective: "Compile MCP definitions into skill + script format"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Create a working skill that replaces direct MCP tool calls"
  - objective: "Validate token reduction through before/after comparison"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Measure and report token savings"

# Cognitive load tracking
cognitive_load:
  new_concepts: 5
  assessment: "5 concepts (MCP token bloat, introspection, code execution pattern, skill compilation, progressive disclosure optimization) - within B1-B2 limit"

# Differentiation guidance
differentiation:
  extension_for_advanced: "Compile multiple MCP servers into a unified skill library; implement caching strategies"
  remedial_for_struggling: "Focus on single MCP server; use provided templates"

# Generation metadata
generated_by: "content-implementer v1.0.0 (045-lesson-09-compiling-mcp-skills)"
source_spec: "specs/045-lesson-09-compiling-mcp-skills/spec.md"
created: "2025-12-19"
last_modified: "2025-12-19"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"

# Legacy compatibility
prerequisites:
  - "Lesson 05: The Concept Behind Skills"
  - "Lesson 06: Agent Skills (SKILL.md format)"
  - "Lesson 08: MCP Integration"
---

# Compiling MCP to Skills

You've learned MCP servers connect Claude Code to external systems. You've mastered skills as encoded expertise. Now: what happens when you want both, but MCP's token consumption makes it expensive?

![skills-mcp](https://pub-80f166e40b854371ac7b05053b435162.r2.dev/books/ai-native-dev/static/images/part-2/chapter-05/skills-mcp.png)

This lesson shows you a powerful pattern: **compile high-token MCP servers into lean skills**, reducing context consumption by up to 98.7% while maintaining full functionality.

:::tip Industry Standard
Skills format is now supported by Claude Code, OpenAI Codex (beta), and Goose. 
Skills you compile here work across all three agents.
:::

---

## The Problem: MCP Token Bloat

When Claude Code loads an MCP server, it eagerly loads ALL tool definitions. A single server like Sentry loads **8,000+ tokens of documentation immediately**—before you've asked a single question.

Here's the real impact. Imagine a workflow that:
1. Calls an MCP tool to fetch data (8,000 tokens for definitions)
2. Processes the data
3. Calls another MCP tool (another 8,000 tokens)
4. Processes again
5. Calls a third tool (another 8,000 tokens)

In a 2-hour meeting transcript processing workflow, you might consume **50,000+ additional tokens** just from repeated MCP tool definition loading—pure waste.

> "The description is both too long to eagerly load it, and too short to really tell the agent how to use it."
> — Armin Ronacher, [Skills vs Dynamic MCP Loadouts](https://lucumr.pocoo.org/2025/12/13/skills-vs-mcp/)

Across 100 MCP-heavy projects, **80-98% of token consumption could be eliminated** through compilation (Anthropic, 2025).

#### 💬 AI Colearning Prompt
> "I have 3 MCP servers installed. Help me estimate my token overhead: For each server, how many tokens does it load at startup? What's my total context cost before I've even asked a question?"

---

## The Solution: Code Execution Pattern

Instead of calling MCP tools directly through Claude's context, use this architecture:

1. **Introspect** the MCP server once (ask Claude: "What tools does this server provide?")
2. **Compile** tool definitions into a lean SKILL.md (~100 tokens, loaded once)
3. **Generate** executable scripts that call the MCP tools locally
4. **Apply** progressive disclosure—only load the skill when relevant, only execute needed scripts

**The math**: 150,000 tokens → 2,000 tokens (98.7% reduction).

From Anthropic's research:
- Standard MCP loading: ~8,000 tokens per server
- Compiled skill format: ~100-200 tokens
- Token savings: 95-98% per tool
- Additional benefit: Local execution avoids repeated context re-loading

---

## Hands-On: Introspect an MCP Server

You'll compile the Playwright MCP (from Lesson 08) into a skill. The process has three steps: introspect, compile, validate.

### Step 1: Ask Claude to Introspect

Start Claude Code in your project:

```bash
claude
```

Ask Claude to analyze the Playwright MCP:

```
The Playwright MCP is installed on my system.
Use it to tell me: What tools does Playwright provide?
For each tool, describe: name, what it does, main parameters, typical output.
Keep descriptions concise (2-3 sentences per tool).
```

Claude will call Playwright MCP and extract its tool definitions. You'll get output like:

```
Playwright MCP Tools:

1. browser_navigate
   - Navigates to a URL in the browser
   - Parameters: url (string), timeout (optional)
   - Output: Page title, loaded status

2. browser_click
   - Clicks an element on the current page
   - Parameters: element (selector), button (left/right/middle)
   - Output: Success/failure status, new page state

3. browser_take_screenshot
   - Takes a screenshot of the current page
   - Parameters: fullpage (true/false)
   - Output: Base64-encoded image

[... more tools ...]
```

**Troubleshooting Introspection:**

| Problem | Solution |
|---------|----------|
| "MCP not found" | Run `claude mcp list` to verify installation |
| Output truncated | Ask Claude to "Continue with remaining tools" |
| Token estimates seem high | Normal—this justifies compilation |
| No tools returned | Check MCP server is running and accessible |

### Step 2: Compile to Skill Format

Now tell Claude to compile these definitions into a SKILL.md:

```
Based on the Playwright tools you just introspected, create a SKILL.md file
for browser automation. The skill should:

1. Have YAML frontmatter with: name, description, version
2. Include "When to Use" section listing trigger conditions
3. Include "Procedure" section with step-by-step browser workflow patterns
4. Include "Output Format" showing what results look like
5. Be lean (under 300 tokens) but complete enough for autonomous activation

Format it exactly like the blog-planner skill from Lesson 06.
Use this directory structure:
.claude/skills/browser-automation/SKILL.md
```

Claude will generate the compiled SKILL.md. You copy it to your project. Here's what a compiled skill looks like (abbreviated):

```yaml
---
name: "browser-automation"
description: "Automate web browsing tasks: navigate to pages, find elements, extract data, take screenshots. Use when you need to interact with web interfaces programmatically."
version: "1.0.0"
---

# Browser Automation Skill

## When to Use This Skill

- User needs to navigate to a URL and extract information
- User wants to automate clicking buttons or filling forms
- User asks to take a screenshot of a webpage
- User needs to test web application behavior

## Procedure

1. **Navigate to page**: Load the target URL
2. **Locate element**: Use selectors (CSS or XPath) to find target elements
3. **Interact**: Click, type text, or extract data
4. **Validate**: Take screenshot or verify result
5. **Extract output**: Format results (links, text, images) for user

## Output Format

**For data extraction**:
- List items found with relevant attributes (href, text, price)
- Include any metadata (page load time, errors encountered)

**For screenshots**:
- Base64 image or visual description

**For interaction**:
- Success/failure confirmation
- New page state or loaded content
```

### Step 3: Test the Compiled Skill

Place the SKILL.md in your `.claude/skills/browser-automation/` folder:

```bash
mkdir -p .claude/skills/browser-automation
# Paste the generated SKILL.md into that folder
```

Now test it. In Claude Code, ask:

```
I've created a browser-automation skill. Test it by navigating to https://example.com
and extracting the main heading text. Confirm that the skill activated and worked correctly.
```

Claude will load the skill (not the raw Playwright MCP), execute the necessary browser commands, and return results.

**What happened**: You replaced 8,000 tokens of MCP definitions with 150 tokens of lean skill instructions. Same functionality, drastically less context consumption.

**Troubleshooting Compilation:**

| Problem | Solution |
|---------|----------|
| Skill not discovered | Check path: `.claude/skills/[name]/SKILL.md` (not flat file) |
| Missing YAML frontmatter | Must have `name`, `description`, `version` fields |
| Description too vague | Use formula: Action + Input + Output + Triggers |
| Skill never activates | Make description more specific to your use case |

#### 💬 AI Colearning Prompt
> "Review the SKILL.md I just created. Is the description specific enough to trigger reliably? What edge cases might cause it to activate incorrectly or fail to activate when needed?"

---

## The Compilation Workflow: Spec→Prompt→Script→Validation

Let me show you a complete compilation for a more complex MCP server (Context7, from Lesson 08).

### Step 1: Specification (Intent)

Before compiling, understand what you're compiling FOR:

```
MCP Server: Context7
Goal: Fetch latest documentation for any library
Current Token Cost: 6,000 tokens (full tool definitions)
Target Token Cost: 100-150 tokens (compiled skill)
Use Case: "Get current docs for React 19" queries, not one-off lookups
```

### Step 2: Introspection Prompt (Extract Definitions)

```
I have Context7 MCP installed. Walk me through:
1. What tools does it provide? (List names + one-line descriptions)
2. What parameters does each tool accept?
3. What does typical output look like?

Be concise. I want to compile this into a skill, so focus on the essential information
a skill user needs to activate this tool correctly.
```

Claude introspects and returns the tool catalog.

### Step 3: Compilation Prompt (Generate Skill)

```
Based on the Context7 tools you just analyzed, generate a SKILL.md file that:

1. Has name "documentation-fetcher"
2. Has description that explains when Claude should activate this skill
   (when user asks about current library docs, requires up-to-date info)
3. Includes "When to Use" triggers for documentation research queries
4. Includes "Procedure" for:
   - Parsing the library/version from user request
   - Executing the appropriate Context7 tool
   - Formatting results with citations and URLs
5. Includes "Output Format" showing structured documentation results

Keep it under 200 tokens. Make it detailed enough that Claude can use it
without calling you back with clarifications.

Format as SKILL.md with YAML frontmatter.
```

Claude generates the skill. You save it to `.claude/skills/documentation-fetcher/SKILL.md`.

### Step 4: Script Generation (Optional, for Repeated Calls)

For complex workflows that call the MCP repeatedly, generate a reusable script:

```
I'm compiling Context7 into a skill. I also want a TypeScript script
that I can call from other contexts (CLI, cron jobs, etc.)
to fetch documentation without loading Claude's context.

Generate a TypeScript script that:
1. Takes library name + version as arguments
2. Calls the Context7 API (or equivalent) to fetch docs
3. Formats results as JSON
4. Outputs to stdout

This script should be in .claude/skills/documentation-fetcher/fetch-docs.ts
```

Claude generates the script. You save it alongside SKILL.md.

### Step 5: Validation (Measure Impact)

Compare token consumption:

**Before compilation** (direct MCP):
```
MCP server loaded: 6,000 tokens
Context7 definitions: 4,000 tokens
Tool invocation overhead: 1,000 tokens
Total per query: ~11,000 tokens
```

**After compilation** (skill + script):
```
SKILL.md loaded: 150 tokens (once per session)
Script execution: 0 tokens (runs locally, no context)
Tool invocation overhead: 200 tokens
Total per query: ~200-350 tokens
```

**Savings**: 11,000 → 350 tokens (96.8% reduction).

### Reflection: Was It Worth It?

Before moving on, consider:

- **Was the compilation effort worth the token savings?** For a one-off query, probably not. For a workflow you run daily, absolutely.
- **What would make compilation NOT worth it?** Rapidly changing APIs, one-time lookups, very low-token MCP servers.
- **How did your understanding change?** You now see MCP tools as raw material to be refined, not final products.

---

## Using Skill-Creator to Automate Compilation

Writing introspection prompts manually is tedious. The `skill-creator` meta-skill (from Lesson 06) can automate MCP compilation.

### Workflow: Automate with Skill-Creator

Open Claude Code in the skills lab:

```bash
cd [path-to-claude-code-skills-lab]
claude
```

Ask skill-creator to help:

```
I want to compile an MCP server into a skill.
The server is: [MCP server name]
The goal is: [what I use it for]
The repetition: [how often do I call this?]

Use the skill-creator to walk me through:
1. Introspecting the MCP server
2. Designing the SKILL.md structure
3. Generating the complete skill file
4. Testing it in my project

I've already installed the MCP server (run "claude mcp list" to verify it's available).
```

Skill-creator will:
1. Guide you through questions about your MCP server
2. Help you introspect its tools
3. Generate the complete SKILL.md
4. Suggest test prompts to verify it works

The skill-creator output is production-ready SKILL.md in the correct format.

---

## Decision Framework: When to Compile vs. Use Direct MCP

Not every MCP server needs compilation. Use this matrix to decide:

| Scenario | Recommendation | Reasoning |
|----------|----------------|-----------|
| **One-off query** | Use MCP directly | Compilation overhead not worth it for single use |
| **Repeated workflow** (3+ times) | Compile to skill | Amortizes compilation cost across multiple uses |
| **High token definitions** (5,000+ tokens) | Compile to skill | Token savings justify upfront work |
| **Low token definitions** (<500 tokens) | Use MCP directly | Compilation provides minimal benefit |
| **Rapidly changing API** | Use MCP directly | Compiled skill becomes stale quickly |
| **Stable tool set** | Compile to skill | Skill remains accurate over time |
| **Privacy-sensitive data** | Compile to skill | Local script execution avoids context logging |
| **Integration with other skills** | Compile to skill | Composability improves with skill format |
| **Team workflow** | Compile to skill | Shareable SKILL.md vs proprietary MCP setup |

**Decision shortcut**:
- Calling MCP 1-2 times? → Use direct MCP
- Calling MCP 3+ times in same session? → Compile to skill
- Working with high-token server? → Compile to skill
- Building production workflow? → Compile to skill

---

## What's Ahead

You've learned to compile MCP servers into lean skills—reducing token consumption by 80-98% while preserving functionality. But what happens when you need multiple skills working together on complex tasks?

**Lesson 10: Subagents and Orchestration** introduces the next level: specialized AI assistants that handle specific types of work with isolated context. Where compilation gives you *efficiency*, subagents give you *coordination*—a team of focused specialists instead of one overloaded generalist.

Skills you compile now become building blocks for subagent workflows later.

---

## Sources

Research and tools supporting this lesson:

- [Anthropic: Code Execution with MCP](https://www.anthropic.com/engineering/code-execution-with-mcp) — Architecture for local execution + context reduction
- [Armin Ronacher: Skills vs Dynamic MCP Loadouts](https://lucumr.pocoo.org/2025/12/13/skills-vs-mcp/) — Token efficiency analysis and pattern recommendations
- [SmartScope: MCP Code Execution Deep Dive](https://smartscope.blog/en/blog/mcp-code-execution-agent-design/) — Detailed compilation workflow examples
- [Claude Code Documentation: MCP Integration](https://docs.anthropic.com/claude-code/mcp) — Official MCP protocol reference

---

## Try With AI

**Introspect and Compile Your First MCP:**

> "I have the Playwright MCP installed (or Context7 MCP). Walk me through compiling it to a skill: (1) Introspect the server to extract tool definitions, (2) Generate a complete SKILL.md in the correct format, (3) Show me exactly where to save it in my project, (4) Give me a test prompt to verify it works."

**Automate with Skill-Creator:**

> "Use the skill-creator to guide me through compiling [specific MCP server] to a skill. Start by asking me clarifying questions about my use case, then generate the complete SKILL.md and any supporting scripts. Make the output production-ready."

**Measure Token Savings:**

> "I've compiled an MCP server to a skill. Help me measure the token savings: (1) Show me how to log token counts before/after, (2) Calculate the percentage reduction, (3) Determine if the compilation was worthwhile for my use case, (4) Suggest optimizations if needed."

**Build a Skill Suite from Multiple MCPs:**

> "I work with multiple MCP servers (list them). Design a skill suite that composes them efficiently: For each server, should I compile to a skill or use direct MCP? If compiling, what should each skill do? How should they work together? Show me the folder structure and SKILL.md templates."

**Design a Script for Standalone Execution:**

> "I've compiled an MCP server to a skill. Now I want a TypeScript/Python script that calls this MCP outside of Claude Code (in a cron job, API endpoint, etc.). Generate a script that: (1) Accepts parameters matching the skill's inputs, (2) Executes the MCP functionality, (3) Formats output as JSON, (4) Can be called from CLI."
