---
sidebar_position: 5
title: "Connecting MCP Servers and Common Workflows"
---

# Connecting MCP Servers and Common Workflows

## Claude Code as Your Collaborative Partner (Beginner-Friendly)

Think of Claude Code like a helpful teammate sitting beside you. You ask for what you want in plain language, and Claude Code helps you do it—search the web, read docs, and work with tools—without needing to write code.

By default, Claude Code only sees your local files. But much of what you need lives elsewhere: websites, docs, APIs. **Model Context Protocol (MCP)** is the bridge that lets Claude Code safely use external tools and data—so it can truly collaborate with you on real tasks.

In this lesson, you will:
- Understand what MCP is (in simple terms)
- Add two beginner-friendly MCP servers to Claude Code
  - **Playwright MCP**: lets Claude browse and extract information from websites
  - **Context7 MCP**: gives Claude instant access to up-to-date library and API docs
- Try two real workflows you can copy/paste and run immediately

No programming experience required.

---

## What Is Model Context Protocol (MCP)?

**In simple terms**: MCP is how Claude Code safely uses tools outside your computer. Each tool is an "MCP server" (for example: a web browser or a docs helper).

### How MCP Works: The Architecture

**Without MCP**:
```
You ↔ Claude Code ↔ Local Files Only
```

**With MCP**:
```
You ↔ Claude Code ↔ MCP Servers ↔ External Systems
                        ↓
              [Web browsing, Docs search]
```

**MCP Server**: A small helper app Claude uses to do a job (browse the web, fetch docs, query a database).

**MCP Connection**: The saved info Claude needs to talk to that helper.

### MCP vs. Skills vs. Subagents

You've now learned three extension mechanisms. Here's how they differ:

| Feature | Subagent | Agent Skill | MCP Server |
|---------|----------|-------------|------------|
| **Purpose** | Specialized AI assistant | Autonomous expertise | External data access |
| **Data Source** | Local files only | Local files only | **External systems** |
| **Example** | Code reviewer with custom standards | Auto-generate docstrings | Query GitHub issues, access database |
| **Invocation** | Explicit: `claude agent run` | Autonomous discovery | Automatic when relevant |
| **Security Concern** | Low (local files) | Low (local files) | **High (external access)** |

**Key Distinction**: MCP servers give Claude Code access **beyond your local machine**, which is powerful but requires careful security evaluation.

---

## A Note on Security (Read This First)

**Stay safe**:
- Use trusted MCP servers. In this lesson we’ll use two widely used, reputable servers: Playwright MCP and Context7 MCP.
- Your tokens and secrets are stored in your system keychain (not plain text).
- Never paste secrets into files; use prompts when Claude asks or environment variables.

---

## Hands-On: Add Two Helpful MCP Servers

We’ll add two servers using simple commands.

```bash
# 1) Playwright MCP (browse the web)
claude mcp add --transport stdio playwright npx @playwright/mcp@latest

# 2) Context7 MCP (get up-to-date docs)
claude mcp add --transport stdio context7 npx @upstash/context7-mcp
```

---

## Workflow 1: Shop Together — Find a Shirt on Amazon (Playwright MCP)

Setup (one-time):

```bash
# Add Playwright MCP (web browsing)
claude mcp add --transport stdio playwright npx @playwright/mcp@latest

# Add Context7 MCP (live docs)
claude mcp add --transport stdio context7 npx @upstash/context7-mcp
```

Goal: Ask Claude to browse Amazon and find a shirt that matches your preferences. No code—just a plain request.

In Claude Code, say:

```
Use the Playwright MCP to browse Amazon. Find 3 men's casual shirts under $30 with good reviews. Share links, prices, main features, and any sizing notes. Prefer neutral colors.
```

What happens:
- Claude launches the Playwright MCP to visit Amazon
- It navigates pages, extracts details, and returns a neat summary with links
- You can iterate naturally: “filter to long-sleeve” or “show only Prime-eligible”

If you get an error:
- Ensure `playwright` MCP is registered
- Try again; websites change often, so Claude may adjust its browsing steps

---

## Workflow 2: Learn What’s New — Ask for MCP Docs (Context7 MCP)

Goal: Ask Claude to use Context7 to fetch and summarize the latest resources about MCP in Claude Code.

In Claude Code, say:

```
Use the Context7 MCP to fetch the latest official documentation and articles about MCP support in Claude Code. Summarize what MCP is, how to add a server, and any recent changes or best practices. Include links and short quotes for key points.
```

What happens:
- Claude queries Context7’s knowledge sources for up-to-date docs
- You get a short, current summary with citations and links
- Ask follow-ups: “show the exact CLI command to add a server via stdio” or “compare Context7 MCP vs GitHub MCP”

Tip: This is your “know about anything new” button. Use it anytime you need the latest docs without hunting across websites.

---

## The Complete Claude Code Toolkit

You now have four extension mechanisms:

1. **Main Conversation**: Exploratory, flexible, one-off tasks
2. **Subagents**: Specialized, repeatable, context-isolated tasks
3. **Skills**: Autonomous expertise automatically applied
4. **MCP Servers**: External tools and data (web, docs, APIs)

**Strategic Decision Framework**:

```
Need external tools/data (web, docs, database)?
  → Use MCP Server

Have repetitive task with clear rules?
  ├─ Want automatic application → Create Agent Skill
  └─ Want explicit control → Create Subagent

Exploratory or one-off task?
  → Use main conversation
```

### Reflection Questions

**Understanding**:
- Can you explain the difference between subagents (explicit), skills (autonomous), and MCP (external data) to a colleague?
- How does MCP extend Claude Code's capabilities beyond just reading local files?

**Application**:
- What websites or shops would you explore first with Playwright MCP?
- How could Context7 help you keep up with fast-changing tools and libraries?

**Integration**:
- How would you combine a subagent, a skill, and MCP in a single project?
- If your subagent needed external data, how could you use MCP to provide it?

**Practical Experience**:
- What was the difference between explicitly invoking a subagent vs. having a skill automatically discover when to help?
- How did using MCP feel different from just asking Claude Code about local files?

---

## Try With AI

Use Claude Code for this activity (preferred, since you just installed it). If you already have another AI companion tool set up (e.g., ChatGPT web, Gemini CLI), you may use that instead—the prompts are the same.

### Prompt 1: MCP Troubleshooting

```
I'm trying to add an MCP server to Claude Code and it's not working. I ran [paste your command]. The error says [paste error message]. Walk me through troubleshooting: (a) What's the most likely cause? (b) What should I check? (c) Give me 3 diagnostic commands to run. (d) If those fail, what's plan B?
```

**Expected outcome:** Troubleshooting guidance for MCP connection issues

### Prompt 2: Safe Testing Workflows

```
I successfully added Playwright MCP and Context7 MCP. Now I want to test them safely. Create 3 'Hello World' workflows for me: (a) One using Playwright to browse a safe website, (b) One using Context7 to fetch docs for [my library/framework], (c) One combining BOTH MCPs in a single workflow. Include the exact prompts I should give Claude Code.
```

**Expected outcome:** Safe, tested workflows you can run immediately

### Prompt 3: Security Boundaries

```
The lesson emphasizes MCP security concerns. I'm nervous about external access. Help me establish safe boundaries: (a) What types of MCP servers should I AVOID as a beginner? (b) What permissions are risky? (c) How do I audit what an MCP server can access? (d) Create a 'MCP safety checklist' I can follow.
```

**Expected outcome:** Security boundaries and audit procedures

### Prompt 4: Complete Workflow Design

```
Now that I understand subagents, skills, AND MCP servers, help me design a complete workflow: I want to [describe your goal: research a topic / build a feature / debug an issue]. Design a workflow that uses: (a) the right subagent (or main conversation), (b) a relevant skill (if applicable), (c) the right MCP server(s). Show me step-by-step what I'd do.
```

**Expected outcome:** Complete workflow design combining all Claude Code features
