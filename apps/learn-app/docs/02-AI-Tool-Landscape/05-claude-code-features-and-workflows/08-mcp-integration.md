---
title: "MCP Integration"
sidebar_position: 8
chapter: 5
lesson: 8
duration_minutes: 14

# PEDAGOGICAL LAYER METADATA
primary_layer: "Layer 2"
layer_progression: "L2 (AI Collaboration)"
layer_1_foundation: "MCP protocol basics, external system concepts"
layer_2_collaboration: "Co-exploring MCP servers with AI (AI suggests use cases, student provides application context, convergence on safe integration patterns)"
layer_3_intelligence: "N/A (MCP servers themselves are reusable, but lesson teaches usage not creation)"
layer_4_capstone: "N/A"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Configuring MCP Servers for External Access"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety and Security"
    measurable_at_this_level: "Student can understand MCP as standardized external access protocol, install and configure MCP servers (Playwright for web browsing, Context7 for documentation), execute workflows integrating external data, and apply security best practices when enabling external access"

learning_objectives:
  - objective: "Understand Model Context Protocol (MCP) as safe external system access for Claude Code"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of how MCP enables Claude Code to access web, databases, APIs safely vs direct access"
  - objective: "Install and configure Playwright MCP for web browsing capabilities"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful installation and test of Playwright MCP with web browsing task"
  - objective: "Install and configure Context7 MCP for up-to-date documentation retrieval"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful installation and retrieval of current library documentation"
  - objective: "Execute practical workflows: browsing websites and fetching current docs"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Completion of workflow combining MCP with Claude Code task execution"
  - objective: "Apply security best practices when enabling external MCP server access"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Explanation of MCP security model and appropriate vs inappropriate MCP uses"

# Cognitive load tracking
cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (MCP protocol, external access, Playwright MCP, Context7 MCP, security model, workflow integration) - within B1 limit of 10 ✓"

# Differentiation guidance
differentiation:
  extension_for_advanced: "Create custom MCP server for team-specific needs (database access, internal APIs), explore GitHub MCP, Filesystem MCP, configure multiple MCP servers with different security profiles"
  remedial_for_struggling: "Focus on single MCP server (Playwright only), use pre-configured examples, skip Context7 initially"

# Generation metadata
generated_by: "content-implementer v1.0.0 (029-chapter-5-refinement)"
source_spec: "specs/029-chapter-5-refinement/spec.md"
created: "2025-01-17"
last_modified: "2025-01-17"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"

# Legacy compatibility (Docusaurus)
prerequisites:
  - "Lessons 01-07: Claude Code working, skills, CLAUDE.md understanding"
  - "Understanding of external APIs and web services"
---

# MCP Integration: Connecting to External Systems

Right now, Claude Code can only see **files on your computer**.

But what if you need Claude to:
- Browse a website to find information?
- Check the latest documentation for a library?
- Query a database?
- Access an API?

All of that lives **outside your computer**. Claude Code can't reach it... yet.

**Model Context Protocol (MCP)** solves this problem. It's like giving Claude Code safe, approved access to the outside world.

![skills-mcp-complementarity](https://pub-80f166e40b854371ac7b05053b435162.r2.dev/books/ai-native-dev/static/images/part-2/chapter-05/skills-mcp-complementarity.png)

---

## Think of MCP Like This

Imagine Claude Code is a brilliant assistant who works in your office (your computer).

**Without MCP**: Your assistant can only use what's in the office—files on your desk, folders in your cabinet. That's it.

**With MCP**: You give your assistant a **phone directory** with approved contacts—a web browser expert, a documentation specialist, a database consultant. Now when your assistant needs outside information, they can call the right expert and get answers safely.

**MCP is that phone directory.** It connects Claude Code (your AI agent) to external tools and data sources in a **standardized, safe way**.

---

## Skills + MCP: Expertise Packs Meet Data Pipes

Here's a powerful mental model from Anthropic:

| Component | Role | Analogy |
|-----------|------|---------|
| **Skills** | The "How-To" — expertise packs | Teaching Claude a specific workflow |
| **MCP** | The "With-What" — data pipes | Connecting skills to live data |

**Skills are Expertise Packs**: They encode your procedures, your quality criteria, your domain knowledge. "How to analyze financial statements according to our Q4 risk framework."

**MCP provides Data Pipes**: They connect those expertise packs to live data sources. Your SQL database. Your Jira board. The web.

**Together**: Claude knows HOW to do something (skill) and has access to WHAT it needs (MCP).

**Example**: A skill encodes your company's financial reporting procedures. An MCP server connects Claude to your accounting database. Result: automated reports that follow your exact standards using real-time data.

---

## Why This Matters: Safe External Integration

**Workflow Impact**: MCP gives Claude Code safe, standardized access beyond your local files. Without it, AI is limited to your codebase. With it, AI can browse documentation, fetch real-time data, interact with external services—all through secure, permission-controlled protocols.

**Paradigm Connection**: This is where AI-driven development becomes truly powerful. CLAUDE.md (Lesson 07) gives AI your project context. MCP gives AI access to the *entire world* of external knowledge and tools—documentation, APIs, databases, web content—while maintaining security boundaries.

**Real-World Context**: When building production applications, you'll use MCP to:
- Fetch latest library documentation (Context7 MCP)
- Test web interfaces automatically (Playwright MCP)
- Query databases safely (PostgreSQL MCP)
- Access internal company APIs (custom MCP servers)

MCP is the bridge between local context and external intelligence.

---

## What Is Model Context Protocol (MCP)?

**Simple definition**: MCP lets Claude Code safely use external tools through standardized connections.

**An "MCP server"** is a helper tool Claude can call:
- **Playwright MCP**: Lets Claude browse websites
- **Context7 MCP**: Lets Claude fetch up-to-date documentation
- **GitHub MCP**: Lets Claude query GitHub repositories
- **Database MCP**: Lets Claude query databases

Think of each MCP server as a **specialist** Claude Code can consult when needed.

#### 💬 AI Colearning Prompt
> "Explain how MCP provides safe external access compared to direct API calls. What security model does MCP use to protect user data?"

---

## In This Lesson

You will:
1. Add two beginner-friendly MCP servers to Claude Code
2. Try real workflows: browse Amazon, fetch docs
3. Understand when you need external access

No programming experience required. Just copy, paste, and see it work.

## A Note on Security (Read This First)

**Why security matters with MCP**: Unlike working with local files, MCP servers can access the internet, APIs, and external systems. This is powerful but requires trust.

**Stay safe**:
- Use trusted MCP servers. In this lesson we'll use two widely used, reputable servers: Playwright MCP and Context7 MCP.
- Your tokens and secrets are stored in your system keychain (not plain text).
- Never paste secrets into files; use prompts when Claude asks or environment variables.

---

## Hands-On: Add Two Helpful MCP Servers

We'll add two servers using simple commands.

```bash
# 1) Playwright MCP (browse the web)
claude mcp add --transport stdio playwright npx @playwright/mcp@latest

# 2) Context7 MCP (get up-to-date docs)
claude mcp add --transport stdio context7 npx @upstash/context7-mcp
```

**What's happening**: You're telling Claude Code about two external helpers it can use. Each command registers an MCP server that Claude can invoke when relevant.

---

## Workflow 1: Shop Together — Find a Shirt on Amazon (Playwright MCP)

Goal: Ask Claude to browse Amazon and find a shirt that matches your preferences. No code—just a plain request.

In Claude Code, say:

```
Use the Playwright MCP to browse Amazon. Find 3 men's casual shirts under $30 with good reviews. Share links, prices, main features, and any sizing notes. Prefer neutral colors.
```

What happens:
- Claude launches the Playwright MCP to visit Amazon
- It navigates pages, extracts details, and returns a neat summary with links
- You can iterate naturally: "filter to long-sleeve" or "show only Prime-eligible"

If you get an error:
- Ensure `playwright` MCP is registered: `claude mcp list`
- Try again; websites change often, so Claude may adjust its browsing steps
- Verify your internet connection is stable

---

## Workflow 2: Learn What's New — Ask for MCP Docs (Context7 MCP)

Goal: Ask Claude to use Context7 to fetch and summarize the latest resources about MCP in Claude Code.

In Claude Code, say:

```
Use the Context7 MCP to fetch the latest official documentation and articles about MCP support in Claude Code. Summarize what MCP is, how to add a server, and any recent changes or best practices. Include links and short quotes for key points.
```

What happens:
- Claude queries Context7's knowledge sources for up-to-date docs
- You get a short, current summary with citations and links
- Ask follow-ups: "show the exact CLI command to add a server via stdio" or "compare Context7 MCP vs GitHub MCP"

**Tip**: This is your "know about anything new" button. Use it anytime you need the latest docs without hunting across websites.

#### 🎓 Expert Insight
> In AI-native development, you don't memorize documentation URLs or bookmark 47 different API reference sites—you understand WHEN you need authoritative documentation vs community knowledge. MCP servers like Context7 are your just-in-time research partners, not replacements for understanding.

---

## What's Ahead

MCP extends Claude Code's reach from your local filesystem to the entire world of external systems. But Claude Code can do more than just access external tools—it can delegate complex tasks to specialized assistants.

Lesson 09 introduces **subagents**: focused AI specialists that handle specific types of work with isolated context. Where MCP gives Claude Code new *capabilities*, subagents give Claude Code a *team*.

---

## Try With AI

Let's explore MCP integration through hands-on practice with external system access.

**🔍 Explore MCP Capabilities:**

> "I've successfully added the Playwright MCP for web browsing. Show me 3 specific web testing tasks I could accomplish with this MCP right now. For each task, give me the exact prompt I should use and explain what Playwright will do."

**🎯 Practice Building Workflows:**

> "I need to test [describe your specific web application or research goal]. Walk me through building a complete workflow using Playwright MCP (for web browsing) and Context7 MCP (for documentation). Include: which MCP handles which part, exact prompts I should use, and how to verify everything works."

**🧪 Troubleshoot Integration Issues:**

> "I'm trying to add an MCP server and it's not working. I ran [paste your installation command]. The error says [paste error message]. Walk me through troubleshooting: What's the most likely cause? What should I check? Give me 3 diagnostic commands to run with expected outputs."

**🛡️ Establish Security Boundaries:**

> "The lesson emphasizes MCP security. I'm nervous about giving Claude Code external access. Help me establish safe boundaries: What types of MCP servers should I avoid as a beginner? What permissions are risky? How do I audit what an MCP server can access? Create an MCP safety checklist I can follow."
