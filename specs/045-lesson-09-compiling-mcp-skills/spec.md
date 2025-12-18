# Feature Specification: Lesson 09 - Compiling MCP to Skills

**Feature Branch**: `037-lesson-09-compiling-mcp-skills`
**Created**: 2025-12-19
**Status**: Draft
**Input**: User description: "Lesson 09: Compiling MCP to Skills - Teaching students to reduce MCP token bloat by 80-98% through introspection and skill compilation. Based on Anthropic's Code Execution with MCP blog post and Jared Palmer's technique of asking Claude to introspect/compile MCP servers to skill + script files."

## Context

### Chapter Position
- **Chapter 5, Lesson 09** (new lesson inserted between MCP Integration and Subagents)
- **Prerequisites**: Lesson 05 (Concept Behind Skills), Lesson 06 (Agent Skills), Lesson 08 (MCP Integration)
- **Proficiency Level**: B1-B2 (intermediate, applying knowledge from multiple previous lessons)
- **Pedagogical Layer**: L2 (AI Collaboration) - students work WITH Claude to introspect and compile

### Problem Being Solved
MCP servers load ALL tool definitions at startup, consuming thousands of tokens before any work begins. Students who completed Lesson 08 (MCP Integration) installed Playwright and Context7 MCP servers. Now they'll learn to optimize these installations by compiling MCP tool definitions into token-efficient skills.

### Key Sources
- [Anthropic: Code Execution with MCP](https://www.anthropic.com/engineering/code-execution-with-mcp) - 98.7% token reduction pattern
- [Armin Ronacher: Skills vs Dynamic MCP Loadouts](https://lucumr.pocoo.org/2025/12/13/skills-vs-mcp/) - Sentry MCP 8,000 token example
- [SmartScope: MCP Code Execution Deep Dive](https://smartscope.blog/en/blog/mcp-code-execution-agent-design/) - Lazy loading architecture

### Concrete Example for Hands-On
Students already have these MCP servers from Lesson 08:
```bash
claude mcp add --transport stdio playwright npx @playwright/mcp@latest
claude mcp add --transport stdio context7 npx @upstash/context7-mcp
```

They also have the `skill-creator` skill from the Skills Lab (Lesson 04). This lesson teaches them to use Claude + skill-creator to compile one of these MCP servers into an optimized skill.

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Understand the Token Bloat Problem (Priority: P1)

A student who completed Lesson 08 (MCP Integration) notices their Claude Code sessions feel "heavy" or slow. They want to understand WHY their MCP servers consume so many tokens and whether this is normal.

**Why this priority**: Without understanding the problem, students won't appreciate the solution. This creates the "aha moment" that motivates learning the technique.

**Independent Test**: Can be fully tested by asking Claude to show current token usage with MCP servers loaded, and student can articulate the problem in their own words.

**Acceptance Scenarios**:

1. **Given** student has Playwright MCP installed from Lesson 08, **When** they ask Claude "How many tokens are my MCP servers using right now?", **Then** they see a concrete number (e.g., "Playwright MCP tool definitions consume approximately X,XXX tokens") and understand this happens at EVERY session start.

2. **Given** student understands basic token concept, **When** they read the lesson's explanation of MCP token bloat with concrete examples (8,000 tokens for Sentry), **Then** they can explain to a peer why "loading all tools at startup" is wasteful.

3. **Given** student sees the 80-98% reduction claim, **When** they read the explanation of code execution pattern, **Then** they understand the conceptual difference between "calling tools directly" vs "writing code that uses tools on-demand."

---

### User Story 2 - Introspect an MCP Server's Tools (Priority: P2)

A student wants to see exactly what tools their MCP server provides before deciding whether to compile it. They use Claude to "introspect" the server and extract its tool definitions.

**Why this priority**: Introspection is the first technical step. Students must see what's inside before they can compile it.

**Independent Test**: Can be tested by running the introspection prompt and receiving a structured list of tool names, descriptions, and parameters.

**Acceptance Scenarios**:

1. **Given** student has Context7 MCP installed, **When** they ask Claude "Show me all the tools that Context7 MCP provides, including their descriptions and parameters", **Then** they receive a structured list (tool name, description, input parameters) for each tool.

2. **Given** student sees the introspected tool list, **When** they compare tool count and description length to estimated tokens, **Then** they can identify which tools are "high token" (verbose descriptions) vs "low token" (concise).

3. **Given** student has introspected both Playwright and Context7, **When** they compare the two, **Then** they can articulate which server would benefit MORE from compilation (the one with more verbose/numerous tools).

---

### User Story 3 - Compile MCP to Skill Using Skill-Creator (Priority: P1)

A student uses Claude + the skill-creator skill to transform an MCP server's tool definitions into a lean SKILL.md file with supporting scripts. This is the core hands-on exercise.

**Why this priority**: This is the primary learning outcome—actually doing the compilation, not just understanding it theoretically.

**Independent Test**: Can be tested by creating a working skill in `.claude/skills/` that Claude recognizes and activates, replacing direct MCP tool calls.

**Acceptance Scenarios**:

1. **Given** student has introspected Context7 MCP tools, **When** they use skill-creator with the prompt "Create a skill that provides documentation lookup using Context7, optimized for token efficiency", **Then** skill-creator generates a `.claude/skills/docs-lookup/SKILL.md` with:
   - YAML frontmatter (~50-100 tokens)
   - Clear "When to Use" section
   - Lean instructions that reference scripts for execution

2. **Given** skill-creator has generated the skill structure, **When** Claude creates supporting script files, **Then** the scripts (Python or TypeScript) handle the actual Context7 API calls locally, consuming 0 tokens until executed.

3. **Given** the compiled skill exists, **When** student asks Claude "What documentation skills do I have?", **Then** Claude discovers the new skill automatically (progressive disclosure working).

---

### User Story 4 - Validate Token Reduction (Priority: P2)

A student wants to measure the actual token savings achieved by their compiled skill vs direct MCP usage. They run a before/after comparison.

**Why this priority**: Measurement validates the technique worked. Students need proof, not just trust.

**Independent Test**: Can be tested by comparing token usage for the same task using direct MCP vs compiled skill.

**Acceptance Scenarios**:

1. **Given** student has both original MCP and compiled skill available, **When** they ask Claude to perform the same documentation lookup task twice (once via MCP, once via skill), **Then** they can compare approximate token usage and see significant reduction.

2. **Given** student completes the comparison, **When** they calculate the percentage reduction, **Then** they achieve at least 50% token reduction (realistic for beginners; experts achieve 80-98%).

3. **Given** student documents their results, **When** they reflect on the exercise, **Then** they can articulate WHEN compilation is worth the effort vs when direct MCP is acceptable.

---

### Edge Cases

- **What happens when MCP server has too many tools to fit in a single skill?** → Lesson explains splitting into multiple focused skills (e.g., "docs-search" and "docs-fetch" instead of one "docs" skill).
- **What happens when MCP server updates and skill becomes stale?** → Lesson includes "When to recompile" decision framework and version checking approach.
- **What happens when student doesn't have Python installed?** → Lesson provides TypeScript/JavaScript alternative for scripts, or shell script fallback.

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Lesson MUST explain MCP token bloat problem with concrete numbers (e.g., "8,000 tokens for Sentry MCP", "150,000 → 2,000 tokens reduction").
- **FR-002**: Lesson MUST provide step-by-step introspection workflow using Claude prompts that work with Playwright or Context7 MCP from Lesson 08.
- **FR-003**: Lesson MUST demonstrate compilation using skill-creator skill that students already have from Skills Lab (Lesson 04).
- **FR-004**: Lesson MUST produce a working skill in `.claude/skills/` directory that Claude discovers automatically.
- **FR-005**: Lesson MUST include validation exercise comparing token usage before/after compilation.
- **FR-006**: Lesson MUST provide decision framework: "When to compile vs use direct MCP" with clear criteria.
- **FR-007**: Lesson MUST follow Three Roles Framework invisibly (AI teaches introspection pattern, student teaches domain context, convergence on optimized skill).
- **FR-008**: Lesson MUST cite sources (Anthropic blog, Armin Ronacher article) for credibility.
- **FR-009**: Lesson MUST work for students who only have Context7 MCP (simpler) OR Playwright MCP (more complex) installed.
- **FR-010**: Lesson MUST NOT require Python if student doesn't have it—provide JavaScript/TypeScript alternative.

### Key Entities

- **MCP Server**: External tool provider registered with `claude mcp add` (from Lesson 08)
- **Tool Definition**: JSON schema describing an MCP tool's name, description, and parameters
- **Compiled Skill**: SKILL.md + optional scripts that replace direct MCP tool calls
- **Token Count**: Approximate number of tokens consumed by tool definitions at session start

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Student can articulate the MCP token bloat problem and explain why it matters (comprehension check via reflection prompt).
- **SC-002**: Student successfully introspects at least one MCP server and identifies its tools (observable output in Claude conversation).
- **SC-003**: Student creates a working compiled skill that Claude discovers automatically (file exists in `.claude/skills/`, Claude lists it when asked).
- **SC-004**: Student demonstrates token reduction by comparing before/after for the same task (documented comparison showing at least 30% reduction).
- **SC-005**: Student can explain when compilation is worth the effort vs when direct MCP is acceptable (decision framework applied to hypothetical scenarios).

---

## Assumptions

1. **Students have completed Lesson 08**: They have at least one MCP server (Playwright or Context7) installed and working.
2. **Students have skill-creator available**: From Skills Lab in Lesson 04, they have access to the skill-creator skill.
3. **Token counting is approximate**: Claude provides estimates, not exact counts. Lesson uses "approximately" language.
4. **Context7 is simpler for beginners**: Lesson uses Context7 as primary example; Playwright as extension.
5. **Scripts can be Python, TypeScript, or shell**: Lesson provides options based on student's environment.

---

## Non-Goals

- Teaching students to BUILD MCP servers from scratch (that's advanced, future chapter)
- Achieving exactly 98.7% token reduction (that requires advanced optimization; 30-50% is success for beginners)
- Compiling ALL MCP servers (lesson focuses on one as proof of concept)
- Deep dive into MCP protocol internals (stay practical, not theoretical)

---

## Pedagogical Alignment

### Layer 2 (AI Collaboration) Implementation
- **AI as Teacher**: Claude teaches the introspection pattern, suggests skill structure improvements
- **Student as Teacher**: Student provides context about which tools they use most, refines skill descriptions
- **Convergence**: Together they optimize the skill until it works and is token-efficient

### Three Roles Framework (INVISIBLE)
The lesson demonstrates Three Roles through action, not exposition:
- Student asks Claude to introspect → AI teaches what tools exist
- Student says "I mostly use X tool" → Student teaches usage context
- Claude suggests "Let's focus the skill on X" → Convergence toward optimized design

### Cognitive Load
- **New concepts**: 5 (MCP token bloat, introspection, code execution pattern, skill compilation, validation)
- **Assessment**: Within B1-B2 limit, building on prior knowledge from Lessons 04-06 and 08

---

## Lesson Structure (Recommended)

1. **The Problem** (~3 min read)
   - MCP token bloat explained with concrete numbers
   - Quote from Armin Ronacher article
   - "You're paying for tools you're not using"

2. **The Solution** (~3 min read)
   - Code execution pattern from Anthropic blog
   - Skills as "lazy loading" for expertise
   - 80-98% reduction claim with source

3. **Hands-On: Introspect Your MCP Server** (~5 min exercise)
   - Step-by-step prompts to extract tool definitions
   - Identify high-token vs low-token tools
   - Choose which tools to compile

4. **Hands-On: Compile to Skill** (~10 min exercise)
   - Use skill-creator to generate SKILL.md
   - Claude generates supporting scripts
   - Verify skill appears in `.claude/skills/`

5. **Hands-On: Validate Token Reduction** (~5 min exercise)
   - Before/after comparison prompt
   - Calculate percentage reduction
   - Document results

6. **Decision Framework: When to Compile** (~2 min read)
   - Table: Direct MCP vs Compiled Skill scenarios
   - "Compile when..." criteria

7. **Try With AI** (~exploration prompts)
   - Extend to second MCP server
   - Optimize existing skill further
   - Create skill suite for workflow
