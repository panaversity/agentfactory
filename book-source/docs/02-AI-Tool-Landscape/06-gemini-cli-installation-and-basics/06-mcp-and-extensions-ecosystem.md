---
title: "MCP Servers & Business Workflows"
chapter: 2
lesson: 6
estimated_time: "45 minutes"
learning_objectives:
  - "Understand MCP (Model Context Protocol) and why external system access matters for business"
  - "Install and configure Playwright MCP for active web browsing"
  - "Configure Context7 MCP for live API documentation access"
  - "Apply MCP-powered workflows to real competitive research and documentation tasks"
  - "Evaluate security risks when connecting AI to external systems"
sidebar_position: 6
---
# When Your AI Needs More: MCP and Extensions

In previous lessons, you've seen how Gemini CLI can assist with various business tasks using built-in capabilities. However, as your needs grow more complex—like researching multiple competitors or accessing constantly updated documentation—you'll find that the default tools have limitations.

## The Moment You Hit a Wall

You've been using Gemini CLI for business tasks. It reads your sales reports, analyzes competitor data from single web pages, and helps you understand business concepts. But now you encounter limitations:

**What you want but can't do yet:**
- Have your AI actively **browse** competitor websites (clicking through multiple pages, navigating pricing calculators)
- Give your AI access to **constantly updated documentation** (Stripe API changes, Shopify features, industry regulations)
- Connect to your business systems (CRM data, project management tools, live dashboards)
- Pull **real-time information** (stock prices updating every minute, breaking industry news)

The built-in tools (Lesson 3) get you far, but they can't actively browse or access specialized external systems.

This is when you need **MCP** (Model Context Protocol).

---

## Part 1: Understanding MCP — The Foundation

### What Is MCP? (Business Explanation)

**MCP** (Model Context Protocol) is the technology that lets your AI safely connect to external systems and services.

**Simple analogy:**
Think of MCP like electrical outlets in your office:
- **Your computer** (Gemini CLI) has power but needs to connect to external devices
- **The outlet** (MCP) is the safe, standardized connection point
- **External devices** (websites, documentation, databases) plug in through MCP
- **Power flows safely** because the connection follows standards

**Without MCP:**
```
You → Gemini CLI → Your local files only
                    (limited to what's on your computer)
```

**With MCP:**
```
You → Gemini CLI → MCP Servers → External World
                         ↓
              [Live websites, updated docs, databases, APIs]
```

### Why MCP Exists (The Business Problem It Solves)

**Business scenario:** You're researching 10 competitors.

**Without MCP:**
1. You ask Gemini to fetch one competitor's pricing page (built-in web fetch)
2. Gemini reads that single page
3. You manually copy the next competitor's URL
4. Repeat 10 times
5. You manually compile the comparison
6. **Time: 45 minutes to 1 hour**

**With MCP (Playwright server):**
1. You give Gemini a list of 10 competitor URLs
2. Gemini uses the Playwright MCP server to actively browse all 10 sites
3. Gemini navigates through pricing pages, clicks tabs, extracts data
4. Gemini creates a comparison table automatically
5. **Time: 5-10 minutes**

**The difference:** MCP servers give your AI **capabilities** it doesn't have built-in.

### What Is an MCP Server?

An **MCP server** is a small program that runs locally on your computer and gives Gemini access to specific external capabilities.

**Think of MCP servers as specialized tools:**
- **Playwright MCP server**: Web browsing capability (click, navigate, extract data from multiple pages)
- **Context7 MCP server**: Access to constantly updated technical documentation
- **GitHub MCP server**: Query your repositories, issues, pull requests
- **Database MCP server**: Read from your company's database (with proper permissions)


---

## Part 2: Adding MCP Servers to Gemini CLI

Now let's add two powerful MCP servers for business use.

### MCP Server 1: Playwright (Active Web Browsing)

**What it does:** Lets your AI browse websites like a human—clicking links, navigating multiple pages, filling forms, extracting data.

**Business use cases:**
- Competitive pricing research (browse 10+ competitor sites automatically)
- Market research (extract product details from e-commerce sites)
- Job market analysis (browse salary data across multiple job boards)

#### How to Add Playwright MCP Server (AI-Native Way)

Instead of manually editing configuration files, **ask Gemini to help you set it up**.

**Step 1: Ask Gemini how to configure Playwright MCP**

Open Gemini CLI and ask:

```
I want to add the Playwright MCP server to browse websites for competitive research. Generate the exact settings.json configuration for Playwright MCP server. Create a .gemini folder and settings.json and add.
{
  "mcpServers": {
    "playwright": {
      "command": "npx",
      "args": ["@playwright/mcp@latest"]
    }
  }
}
``` 

**Step 2: Verify the configuration (with Gemini's guidance)**

Ask Gemini for OS-specific instructions:
```
How do I verify that the Playwright MCP server is correctly configured on my [Windows/Mac/Linux] system?
```

Gemini will give you commands like:

```bash
# After pasting the config, refresh MCP
gemini /mcp refresh

# Verify
gemini /mcp
```


**Why this approach is better:**
- ✅ You learn by asking, not memorizing
- ✅ Gemini adapts to your OS and situation
- ✅ You can troubleshoot by asking follow-up questions
- ✅ You understand WHY, not just WHAT

### MCP Server 2: Context7 (Live Documentation)

**What it does:** Gives your AI access to constantly updated documentation for popular business tools and APIs.

**Business use cases:**
- Stay current with Stripe API changes
- Learn new tools quickly (Shopify, QuickBooks, Airtable)
- Onboard team members with up-to-date documentation
- Verify compliance requirements (changing regulations)

#### How to Add Context7 MCP Server

**Step 1: Edit your settings.json**

Open the same `.gemini/settings.json` file you created earlier.

**Step 2: Add Context7 to the configuration**

Update the file to include both servers:

```json
{
  "mcpServers": {
    "playwright": {
      "command": "npx",
      "args": ["@playwright/mcp@latest"]
    },
    "context7": {
      "command": "npx",
      "args": ["-y", "@upstash/context7-mcp"]
    }
  }
}
```

**Step 3: Refresh and verify**

```bash
gemini /mcp refresh
gemini /mcp
```

You should now see both `playwright` and `context7` listed.

---

## Part 3: Business Workflows Using MCP Servers

Now that you've added MCP servers, let's use them for real business tasks.

### Workflow 1: Multi-Competitor Pricing Research (Playwright)

**Your goal:** Compare pricing across 5 competitors launching in the next month.

**Traditional approach:**
1. Visit each competitor site manually
2. Navigate to pricing pages
3. Take screenshots or notes
4. Create spreadsheet
5. Compare features
6. **Time: 2-3 hours**

**With Playwright MCP:**

Open Gemini CLI and give this prompt:

```
Use the Playwright MCP server to browse these 5 competitor websites:
1. [Competitor A URL]
2. [Competitor B URL]
3. [Competitor C URL]
4. [Competitor D URL]
5. [Competitor E URL]

For each competitor:
- Navigate to their pricing page
- Extract all pricing tiers (Basic, Pro, Enterprise, etc.)
- Note the monthly/annual costs
- List 3-5 key features for each tier
- Identify any current promotions or discounts

Create a comparison table showing all 5 competitors side-by-side.
Format: [Tier Name] | [Competitor A Price] | [Competitor B Price] | ... | [Key Differentiators]
```

**What Gemini does:**
1. Launches Playwright browser (headless - you won't see it)
2. Visits first competitor site
3. Locates pricing page (clicks navigation if needed)
4. Extracts pricing data
5. Repeats for all 5 competitors
6. Compiles comparison table


**Business value:**
- Comprehensive market intelligence in minutes
- Current pricing (not outdated blog posts)
- Direct comparison for positioning decisions
- Repeatable monthly for ongoing monitoring

### Workflow 2: Understanding API Changes (Context7)

**Your goal:** Your team uses Stripe for payments. Stripe released a new API version. You need to know what changed and if it affects your integration.

**Traditional approach:**
1. Search for Stripe changelog
2. Read through technical release notes
3. Google for explanations
4. Ask developers what it means
5. Compile business impact assessment
6. **Time: 1-2 hours**

**With Context7 MCP:**

```
Use the Context7 MCP server to fetch the latest Stripe API documentation.

I need to understand:
1. What changed in the most recent Stripe API version?
2. Are there breaking changes that affect payment processing?
3. What new features were added?
4. Do existing integrations need updates?
5. What's the timeline for deprecating old endpoints?

Provide a business-friendly summary (not deeply technical) with:
- Impact severity (Critical/Important/Nice-to-know)
- Timeline for action
- Specific recommendations
- Links to official migration guides
```

**What Gemini does:**
1. Queries Context7's indexed Stripe documentation
2. Identifies recent changes
3. Summarizes in business language
4. Provides actionable recommendations


**Business value:**
- Understand technical changes without reading docs yourself
- Clear timeline for action
- Business-language summary for stakeholders
- Prioritized by urgency

### Workflow 3: Quick Tool Evaluation (Context7)

**Your goal:** Your team is considering Airtable for project management. You need a quick assessment of its API capabilities to inform the decision.

**Prompt:**

```
Use Context7 to fetch Airtable API documentation and provide:

1. What can you do with Airtable's API?
   - Read records? Write records? Delete records?
   - Real-time updates or batch only?
   - Automation capabilities?

2. Technical requirements:
   - Authentication method
   - Rate limits (free vs paid tiers)
   - SDK availability (JavaScript, Python, etc.)

3. Business considerations:
   - Is there a free API tier?
   - What's the cost at scale (1000+ API calls/day)?
   - Any limitations that would block our use case?

Give me a 5-minute summary suitable for a non-technical decision-maker.
```

**Business value:**
- Fast tool evaluation (minutes, not hours)
- Technical details translated to business language
- Clear limitations identified upfront
- Informed decision-making

---

## Part 4: Gemini CLI Extensions — The Next Level

Now that you understand MCP servers, let's talk about **Gemini CLI Extensions**.

### What Are Extensions? (Different from MCP Servers)

**MCP servers** = Individual capabilities you add (like Playwright, Context7)
**Extensions** = Pre-packaged bundles that can include:
- MCP servers (already configured for you)
- Custom commands (shortcuts for common tasks)
- Persistent context files (background information Gemini always remembers)
- Configuration templates

**Analogy:**
- **MCP server** = Buying individual apps for your phone
- **Extension** = Downloading a bundle (like "Office Suite" with Word, Excel, PowerPoint pre-configured)

### When to Use Extensions vs. MCP Servers

**Use MCP servers directly when:**
- You need one specific capability (just Playwright, just Context7)
- You want full control over configuration
- You understand what you're adding

**Use Extensions when:**
- Someone packaged a useful workflow for you
- You want multiple related tools pre-configured
- You trust the extension creator
- You want one-command installation

### Example: Security Extension

Imagine a "Business Intelligence" extension that includes:
- Playwright MCP (competitor research)
- Context7 MCP (documentation access)
- Custom `/research` command (pre-configured for market analysis)
- Industry context file (business terminology Gemini knows)

**Installation:**

```bash
gemini extensions install https://github.com/example/business-intelligence
```

One command installs everything.

### How to Discover Extensions

**Official extension gallery:**
Ask Gemini:

```
What Gemini CLI extensions exist for business research and competitive intelligence?
Show me official extensions with good reputations.
```

Gemini will search for reputable extensions and explain what each does.

**Community extensions:**
- GitHub repositories (search "gemini-cli-extension")
- Official Gemini CLI documentation
- Business tool communities (ask what extensions they use)

### Security Considerations for Extensions

**Extensions have more power than individual MCP servers** because they can include:
- Multiple MCP servers
- Custom commands that run code
- Background files that load automatically

**Before installing an extension, ask:**

1. **Who created it?**
   - Official Gemini team? ✅ High trust
   - Reputable company? ✅ Verify their reputation
   - Individual developer? ⚠️ Check activity and reviews

2. **What does it include?**
   - Ask Gemini: "Explain what this extension does and what access it needs"
   - Review the extension's GitHub repository
   - Check for recent updates (maintained = safer)

3. **What data does it access?**
   - Read-only web browsing? ✅ Lower risk
   - Database write access? ⚠️ Higher risk
   - API keys required? ⚠️ Verify security practices

4. **Do you need all its features?**
   - If you only need Playwright, install Playwright MCP directly
   - Don't install bundles "just in case"—add what you need

**Rule of thumb:** Start with individual MCP servers. Add extensions when you trust the creator and need the bundled capabilities.

---

---

## Try With AI

Use Gemini CLI to design a real workflow using MCP servers for your actual business needs.

### Prompt 1: Identify High-Value Use Case
```
I want to save time on this business task: [DESCRIBE YOUR SITUATION]

Examples:
- "I research 10 competitors monthly for pricing and feature changes"
- "I need to stay current with Shopify API updates for our e-commerce platform"
- "I analyze market trends across 20+ industry websites quarterly"

Which MCP servers would help? Playwright? Context7? Something else? Explain why each one helps with my specific task.
```

**Expected outcome**: Identification of which MCP servers address your pain point and why.

### Prompt 2: Design the Complete Workflow
```
For my task from Prompt 1, design a complete MCP-powered workflow:

1. Which MCP servers do I need? (Be specific: Playwright, Context7, etc.)
2. How do I configure them? (Show the settings.json structure)
3. What's my step-by-step process once configured?
4. What prompt do I give you to execute the workflow?
5. What output do I get, and how do I verify it's accurate?
6. How much time does this save vs. doing it manually?

Give me a workflow I can implement and test today.
```

**Expected outcome**: Detailed implementation plan with configuration, commands, and time savings estimate.

### Prompt 3: Security Evaluation
```
Before I implement this workflow, help me evaluate security:

1. What external systems am I giving you access to?
2. What data are you reading? Writing? Modifying?
3. What credentials (if any) are required?
4. What's the worst that could happen if something goes wrong?
5. How do I test this safely with non-sensitive data first?
6. What warning signs should I watch for during testing?

Create a security checklist I must complete before using real business data.
```

**Expected outcome**: Clear risk assessment and safe testing protocol.

### Prompt 4: Implementation & Troubleshooting
```
I'm ready to implement. Walk me through:

1. Exact configuration for my .gemini/settings.json file
2. Commands to verify MCP servers loaded correctly
3. The specific prompt I should give you to start the workflow
4. What success looks like (expected output format)
5. Common errors I might encounter and how to fix them
6. How to disable/remove the MCP server if I change my mind

Assume I'm a beginner—be specific and step-by-step.
```

**Expected outcome**: Complete implementation guide with troubleshooting steps.

