---
sidebar_position: 6
title: MCP Servers & Integration
duration: "20 minutes"
cefr_level: A2
proficiency: Beginner
teaching_stage: 2
stage_name: "AI Collaboration"
cognitive_load:
  concepts_count: 6
  a2_compliant: true
learning_objectives:
  - id: LO1
    description: "Explain why MCP extends AI capabilities beyond local files to external systems"
    bloom_level: "Understand"
    digcomp: "1.3 Managing data, information and digital content"
  - id: LO2
    description: "Install MCP servers (Playwright, Context7) using Gemini CLI commands"
    bloom_level: "Apply"
    digcomp: "3.1 Developing digital content"
  - id: LO3
    description: "Execute web browsing workflows using Playwright MCP to gather information"
    bloom_level: "Apply"
    digcomp: "1.1 Browsing, searching and filtering data"
  - id: LO4
    description: "Retrieve current documentation using Context7 MCP for learning"
    bloom_level: "Apply"
    digcomp: "1.2 Evaluating data, information and digital content"
  - id: LO5
    description: "Apply security best practices when enabling external MCP server access"
    bloom_level: "Apply"
    digcomp: "4.2 Protecting personal data and privacy"
  - id: LO6
    description: "Design workflows combining MCP servers with built-in tools for research tasks"
    bloom_level: "Create"
    digcomp: "3.2 Integrating and re-elaborating digital content"
---

# MCP Servers & Integration

> **A Word Before We Begin**
>
> Imagine hiring an incredibly smart assistant who can write code, answer questions, and solve problems—but they're locked in a room with no phone, no internet, and no access to your company's systems. That's Gemini CLI without MCP servers. In this lesson, you'll see the problem firsthand, then learn how MCP (Model Context Protocol) transforms Gemini from isolated intelligence into a connected powerhouse.

---

## The Problem: AI in Isolation

Let's start with a real demonstration. Launch Gemini CLI:

```bash
gemini
```

Now try this request:

```
I want to learn about beginner-friendly laptops for students.
Browse a product review website and show me:
1. Top 3 recommended laptops for beginners
2. Price range for each
3. Key features that make them good for students

Use whatever tools you have available.
```

**What happens** (Gemini CLI v0.15.0 without MCP):

Gemini will use its **built-in web fetch tool** to try getting this information:

```
🔧 Using tool: web_fetch
   URL: https://www.example-reviews.com/best-student-laptops

📄 Fetching...

I can see the review website, but I'm getting only the basic page structure.
The actual product information (reviews, prices, specifications) loads after
the page opens, which the basic web fetch tool cannot wait for.

Here's what I found on the initial page:
- Page title mentions "Best Laptops for Students 2025"
- There are some navigation menus
- The detailed product reviews haven't loaded yet

The product information you need appears after the page fully loads,
so I cannot access it with the current tools.

**What you can do instead:**
1. Visit the review website manually in your browser
2. Search Google for "best student laptops 2025"
3. Ask me to help you understand what features to look for

Would you like me to help you with any of these alternatives?
```

**The limitation**: Gemini tried to help using its web fetch tool, but:
- ✅ It attempted to fetch the URL
- ❌ Only got initial page content (before full load)
- ❌ Modern websites load content dynamically
- ❌ Can't interact with the page (click buttons, scroll, wait for loading)
- ❌ No direct GitHub API access

**The reality**: Gemini is brilliant but **limited by built-in tools**. It can't:
- ❌ Wait for pages to fully load (only sees initial content)
- ❌ Access websites that require login
- ❌ Browse interactive websites with buttons and forms
- ❌ See content that appears after clicking or scrolling
- ❌ Access specialized databases or private information

### What Gemini CAN Do (Out of the Box)

The built-in tools from Lesson 3 give you:

✅ **File operations**: Read/write files on your local computer
✅ **Shell execution**: Run basic terminal commands
✅ **Web fetching**: Get initial page content from websites
✅ **Search**: Google Search results summaries

These are **basic capabilities**—perfect for many tasks, but limited when you need:
- Websites that require clicking buttons or filling forms
- Content that appears after scrolling or waiting
- Information behind login walls
- Interactive website features
- Specialized databases or tools

### The Gap in Your Workflow

**Real-world scenario**: You're researching online courses to learn photography.

**What you need**:
1. Browse 3 popular online learning platforms
2. Navigate to photography course pages (which load after page opens)
3. Extract course details: price, duration, student ratings
4. Compare which course is best for beginners
5. Make an informed decision

**What Gemini's built-in tools can do**:
- Web fetch gets initial page (but course details don't load)
- You see page structure without the detailed content
- Course details aren't visible yet (still loading)
- You manually visit each site, take notes, compare yourself
- **Time: 45-60 minutes**

This is the **isolation problem**.

---

## The Solution: Model Context Protocol (MCP)

**MCP** (Model Context Protocol) is the **universal bridge** that connects AI tools to external systems.

Think of MCP like **USB for AI**:
- Before USB: Every device needed a custom cable (keyboard cable, mouse cable, printer cable)
- After USB: One standard port connects everything

**Before MCP**:
- ChatGPT builds custom GitHub integration
- Claude builds its own GitHub integration
- Gemini builds yet another GitHub integration
- Result: Duplication, incompatibility, vendor lock-in

**After MCP**:
- One GitHub MCP server works with ChatGPT, Claude, Gemini, and any future AI tool
- Developers build once, use everywhere
- Community creates hundreds of MCP servers (databases, APIs, browsers, custom tools)

### How MCP Works: Simple Explanation

Think of MCP servers as **specialists in a phone directory**:

1. **You ask Gemini a question** that needs external information
2. **Gemini checks its directory** of available MCP servers (specialists)
3. **Gemini calls the right specialist** (e.g., Playwright for web browsing, Context7 for documentation)
4. **The specialist does the work** (browses website, fetches docs, queries database)
5. **The specialist reports back** with the information
6. **Gemini shows you the results** in a readable format

**Example workflow**:

You ask: "What are the top-rated student laptops under $500?"

Behind the scenes:
- Gemini recognizes it needs to browse review websites
- Gemini calls the **Playwright MCP server** (web browsing specialist)
- Playwright opens a browser, navigates to tech review sites, extracts the information
- Playwright returns the data to Gemini
- Gemini shows you: "HP Pavilion 15 ($449, 4.5/5 stars), Lenovo IdeaPad 3 ($399, 4.4/5 stars)"

**Key point**: You don't manage any of this. You just ask questions naturally, and Gemini coordinates with the right specialists automatically.

### MCP Server vs Built-In Tools

| Capability | Built-In Tools | MCP Servers |
|-----------|----------------|-------------|
| **File operations** | ✅ Read/write local files | ✅ Same |
| **Shell commands** | ✅ Run `ls`, `git`, `npm` | ✅ Same |
| **Web fetching** | ✅ Initial page only | ✅ **Full browser automation** (Playwright MCP) |
| **Search** | ✅ Google Search metadata | ✅ **Real-time doc access** (Context7 MCP) |
| **GitHub** | ❌ No access | ✅ **Full API access** (GitHub MCP) |
| **Databases** | ❌ No access | ✅ **SQL queries** (PostgreSQL MCP) |
| **Custom APIs** | ❌ No access | ✅ **Any API** (custom MCP servers) |

### Real-World Example Revisited

**Scenario**: Analyze 10 competitor websites for pricing.

**Without MCP** (built-in web fetch only):
1. Fetch each URL → get initial page only
2. Course details that appear after page load aren't visible
3. Manually visit sites, take notes, compare
4. **Time: 45-60 minutes**

**With Playwright MCP**:
```
Use Playwright to browse these 3 photography course platforms,
navigate to beginner course pages, extract course details, and
create a comparison table.
```

Gemini:
1. Launches browser via Playwright MCP
2. Navigates to each platform, waits for pages to fully load
3. Finds course information, extracts prices, ratings, duration
4. Returns structured comparison table
5. **Time: 3-5 minutes**

---

## Why MCP Is a Breakthrough

Before MCP, every AI tool built isolated integrations:

```
ChatGPT ───> Custom GitHub Plugin (vendor-locked)
           └> Custom Notion Plugin
           └> Custom Slack Plugin

Claude Code ───> Custom GitHub Integration (incompatible)
               └> Custom Notion Integration
               └> Custom Slack Integration

Gemini CLI ───> Custom GitHub Integration (duplicate work)
              └> Custom Notion Integration
              └> Custom Slack Integration
```

**Problems**:
1. **Duplication**: Same integrations built 3+ times
2. **Vendor lock-in**: Switch AI tools → lose all integrations
3. **Limited coverage**: Small teams can't build 100+ integrations
4. **Maintenance burden**: Updates break across multiple codebases

### The MCP Standard

With MCP, **one server works everywhere**:

```
                    GitHub MCP Server
                           ↑
        ┌──────────────────┼──────────────────┐
        ↓                  ↓                  ↓
   ChatGPT          Claude Code          Gemini CLI

                    Notion MCP Server
                           ↑
        ┌──────────────────┼──────────────────┐
        ↓                  ↓                  ↓
   ChatGPT          Claude Code          Gemini CLI
```

**Benefits**:
1. ✅ **Build once, use everywhere**: GitHub MCP works with all AI tools
2. ✅ **Community-driven**: Anyone can build MCP servers for any service
3. ✅ **No vendor lock-in**: Switch AI tools without losing integrations
4. ✅ **Open standard**: Not controlled by one company
5. ✅ **Rapid growth**: 100+ MCP servers already exist

#### 💬 AI Colearning Prompt
> "Why do you think MCP is called 'Model Context Protocol' and not 'Model Tool Protocol'? What does 'context' imply about how AI uses these connections?"
>
> **Hint**: Think about the difference between a one-time tool use vs. ongoing context the AI maintains throughout a conversation.

---

## Seeing the Difference: Before and After MCP

Let's make this concrete with real examples you can try.

### Example 1: The Same Task with MCP (The "After" Picture)

Let's see what happens when you have MCP servers installed. Using the **exact same prompt** as the opening example:

**Prompt**:
```
I want to learn about beginner-friendly laptops for students.
Browse a product review website and show me:
1. Top 3 recommended laptops for beginners
2. Price range for each
3. Key features that make them good for students

Use whatever tools you have available.
```

**With Playwright MCP Server** (after installation in Part 2):

```
🔧 Using tool: playwright_navigate
   URL: https://www.example-reviews.com/best-student-laptops

🌐 Launching browser...
📄 Waiting for page to fully load...
✅ Page loaded successfully

Top 3 Student Laptops (2025):

1. **HP Pavilion 15** - $449
   - 15.6" display, great for reading and writing
   - Long battery life (8+ hours)
   - Lightweight and portable
   - Student discount available

2. **Lenovo IdeaPad 3** - $399
   - Budget-friendly option
   - Fast performance for everyday tasks
   - Includes Microsoft Office
   - Good keyboard for typing

3. **Acer Aspire 5** - $499
   - Best value for features
   - Excellent display quality
   - Comfortable for long study sessions
   - 1-year warranty included

💡 All three support video calls, document editing, and web browsing perfectly for student needs.

Would you like me to:
1. Compare these with other options?
2. Check which one has the longest warranty?
3. Find student discount programs?
```

**The transformation**:
- ✅ **Real browser automation** (waits for full page load)
- ✅ **Accurate, current information** from actual review sites
- ✅ **Structured comparison** of laptops, prices, features
- ✅ **Follow-up options** suggested based on what you need
- ✅ **Time saved**: From "go browse manually" → complete answer in seconds

### Example 2: Multi-Step Workflow (Comparing Learning Resources)

**Scenario**: Compare beginner-friendly online courses for digital skills.

**Without MCP** (manual process):
```
You: "Compare 3 popular websites for learning photography as a beginner"

Gemini: "I can provide general guidance, but you'll need to visit the sites
to see current course offerings. You should check:
1. https://www.skillshare.com/photography
2. https://www.udemy.com/photography
3. https://www.creativelive.com/photography

And compare which one suits beginners best."

Your manual work:
1. Visit 3 websites
2. Browse their photography courses
3. Take notes on prices, duration, ratings
4. Compare and decide
⏱️ Time: 20-30 minutes
```

**With Playwright MCP** (automated):
```
You: "Use Playwright to browse Skillshare, Udemy, and CreativeLive photography
beginner courses, extract key details, and create a comparison table."

Gemini: ✅ Launching Playwright...
        ✅ Browsing Skillshare photography courses...
        ✅ Browsing Udemy photography courses...
        ✅ Browsing CreativeLive photography courses...

| Platform     | Course Name                    | Price       | Duration  | Rating |
|--------------|--------------------------------|-------------|-----------|--------|
| Skillshare   | Photography Basics             | $14/month   | 3 hours   | 4.7/5  |
| Udemy        | Complete Photography Masterclass| $79.99      | 20 hours  | 4.6/5  |
| CreativeLive | Fundamentals of Photography    | $39/month   | 15 hours  | 4.8/5  |

Notes:
- Skillshare has monthly subscription model
- Udemy offers one-time purchase, lifetime access
- CreativeLive includes live sessions with instructors

⏱️ Time: 3-4 minutes
```

### The Key Insight

**MCP transforms Gemini from**:
- 🔒 **Advisor** (tells you what to do) →
- 🔓 **Executor** (does it for you)

**Without MCP**: "Here's how you could solve this manually..."
**With MCP**: "Done. Here are the results. What's next?"

#### 🎓 Expert Insight
> The real power of MCP isn't just speed—it's **removing context switching**. Without MCP, you stop your AI conversation, open 5 browser tabs, manually gather data, return to AI, paste results, and continue. With MCP, you stay in one continuous conversation while AI handles the data gathering. This is the difference between **assisted** development (you do the work) and **AI-driven** development (AI does the work).

---

## Part 2: CLI MCP Management Commands

The modern way to add MCP servers is **CLI commands**—not manual JSON editing.

### Adding MCP Servers

```bash
# Add stdio MCP server (runs locally on your computer)
gemini mcp add my-server-name command-to-run --port 8080

# Add HTTP MCP server (remote API)
gemini mcp add --transport http secure-api https://api.example.com/mcp \
  --header "Authorization: Bearer abc123"

# Add SSE MCP server (streaming)
gemini mcp add --transport sse events-api https://api.example.com/sse
```

**Transport types**:
- **Stdio**: Runs locally on your computer
- **HTTP**: Connects to remote server on the internet
- **SSE**: Real-time streaming connections

### Listing MCP Servers

```bash
gemini mcp list
```

**Output**:
```
Connected MCP Servers:
- playwright (stdio) - connected
- context7 (http) - connected
- my-database (stdio) - connecting...
```

Shows status (connected/disconnected/connecting).

### Removing MCP Servers

```bash
gemini mcp remove server-name
```

Removes the server from your configuration.

### CLI vs Manual Configuration

**CLI approach** (recommended for beginners):
```bash
gemini mcp add playwright npx @playwright/mcp@latest
```

Simple, clear, immediate feedback.

**Manual JSON editing** (advanced):
```json
{
  "mcpServers": {
    "playwright": {
      "command": "npx",
      "args": ["@playwright/mcp@latest"]
    }
  }
}
```

More control, but error-prone for beginners. The previous lesson ([Configuration & Settings](./05-configuration-and-settings.md)) covered `settings.json` structure and environment variables if you need manual configuration.

---

## Part 3: Security and Authentication for MCP Servers

### Why Security Matters

**Important**: MCP servers can access the internet and external systems. This is powerful but requires trust and caution.

Think of it like this: When you give your house key to a contractor, you:
- ✅ Use trusted, recommended contractors (not random people)
- ✅ Check their credentials and reviews
- ✅ Know exactly what work they're authorized to do
- ✅ Revoke access when the job is done

**Same principles apply to MCP servers.**

### Security Best Practices

**Before adding any MCP server**:

1. ✅ **Use trusted sources**: In this lesson, we use widely-adopted servers (Playwright, Context7)
2. ✅ **Check what access it needs**: Read descriptions carefully
3. ✅ **Start with minimal permissions**: Add only what you need
4. ✅ **Review regularly**: Remove servers you no longer use

**What Gemini CLI does for you**:
- Stores sensitive tokens in your system keychain (not plain text files)
- Encrypts authentication credentials
- Auto-refreshes tokens so you don't handle them manually
- Shows you which servers are connected: `gemini mcp list`

### OAuth Authentication: `/mcp auth`

Some MCP servers need **authentication** to access protected systems (like your Google Drive or GitHub account).

**When you need OAuth**:
- MCP server accesses your cloud accounts (Google Drive, GitHub)
- API requires login credentials
- Database access is protected

**Using OAuth** (Gemini handles the complexity):

```bash
/mcp auth
```

Lists servers that need authentication.

```bash
/mcp auth my-database-server
```

What happens:
1. Your browser opens to the login page (Google, GitHub, etc.)
2. You log in and approve access
3. Gemini stores the token securely in `~/.gemini/mcp-oauth-tokens.json`
4. Tokens auto-refresh—you never touch them manually

**You don't need OAuth for**:
- Public websites (Playwright browsing Amazon, Wikipedia)
- Open documentation (Context7)
- Local-only tools

### Security Checklist

Before adding any new MCP server, ask:

- [ ] Is this from a trusted source? (Official npm packages, verified GitHub repos)
- [ ] Do I understand what this server can access?
- [ ] Do I actually need this capability right now?
- [ ] Have I checked recent reviews or issues on the server's repository?
- [ ] Can I remove it easily if I don't use it? (Yes: `gemini mcp remove server-name`)

---

## Part 4: Practical Workflows for Learning and Research

### Workflow 1: Research and Compare Products (Playwright)

**Goal**: Compare 3 beginner laptops for students.

**Setup**:
```bash
gemini mcp add playwright npx @playwright/mcp@latest
```

**Prompt**:
```
Use the Playwright MCP server to research student laptops on these review sites:
1. https://www.example-tech-reviews.com/laptops
2. https://www.student-tech-guide.com/best-laptops
3. https://www.beginner-tech.com/laptop-reviews

For each site:
- Find their top recommended laptops for students
- Extract price, specifications, and ratings
- Note any beginner-friendly features
- Check for student discounts

Create a comparison table showing the top 3 options.
```

**Result**: Clear comparison of beginner-friendly laptops in minutes.

### Workflow 2: Learning New Tools (Context7)

**Goal**: Quickly understand a new tool or library you're interested in learning.

**Setup**:
```bash
gemini mcp add context7 npx -y @upstash/context7-mcp
```

**Prompt**:
```
Use Context7 to help me learn about [tool/library name]. I'm a beginner.
1. What is it used for?
2. What are the main features beginners should know?
3. What are common use cases?
4. Are there any recent major updates?
5. What resources are best for learning it?

Give me a beginner-friendly summary with links.
```

**Result**: Current, accurate documentation and learning resources without manually searching multiple websites.

### Workflow 3: Multi-Tool Combination for Research

**Goal**: Research budget-friendly home office setup for remote learning.

```
1. Use Playwright to browse 3 tech review sites for desk, chair, and laptop recommendations
2. Use Context7 to find current information on ergonomic workspace setup
3. Use file operations to save the combined research to a file
4. Create a shopping list with budget breakdown under $500

Combine all MCP capabilities and built-in tools in one workflow.
```

**Key insight**: MCP servers work together with Gemini's built-in tools. One prompt can coordinate multiple capabilities.

---

## Red Flags to Watch

### "MCP server connection failed"
- Check server is running: `gemini mcp list`
- Verify command syntax: `gemini mcp add --help`
- Try manual configuration (see [Configuration & Settings](./05-configuration-and-settings.md) for `settings.json` structure)

### "Authentication failed: Invalid token"
- Re-authenticate: `/mcp auth server-name`
- Check browser OAuth flow completed
- Verify no extra spaces in configuration

### "Playwright timeout: Browser not responding"
- Website may be slow or blocking automation
- Try shorter timeout: Check [Configuration & Settings](./05-configuration-and-settings.md) for timeout configuration
- Test website manually first

### "Context7 not finding documentation"
- Verify MCP server connected: `gemini mcp list`
- Search term may not match indexed docs
- Try more specific queries

---

## Try With AI

**Setup**: Use Gemini CLI for this activity (preferred for this lesson). You may use Claude Code or other AI tools with MCP support if you prefer.

### Prompt 1: Setting Up Your First MCP Server
```
I want to add the Playwright MCP server to browse websites.
Walk me through:
1. Exact command to add it
2. How to verify it's working
3. A test prompt to make sure it's connected
4. What to do if the connection fails
```

**Expected outcome**: Step-by-step setup with verification commands.

**Safety reminder**: Only add MCP servers from trusted sources. Playwright is widely used and maintained by Microsoft.

### Prompt 2: Choosing the Right MCP Server
```
I have this goal: [describe what you want to accomplish]

Examples:
- "Research products on e-commerce websites"
- "Stay current with documentation for tools I'm learning"
- "Gather information from multiple websites for a report"

Which MCP server should I use? Why? How do I set it up?
Explain like I'm a beginner with no programming experience.
```

**Expected outcome**: Specific recommendation with beginner-friendly setup instructions.

### Prompt 3: Safe Workflow Design
```
I want to use MCP servers to help with: [describe your research or learning goal]

Design a safe, beginner-friendly workflow for me:
1. Which MCP server(s) should I use? (Playwright for browsing? Context7 for docs?)
2. What's the exact prompt I should give you?
3. How do I know if it's working correctly?
4. What are the security considerations I should be aware of?

Show me step-by-step with copyable prompts.
```

**Expected outcome**: Complete workflow design with security guidance customized to your needs.

### Prompt 4: Troubleshooting MCP Issues
```
I'm getting this error: [your error message]

After running: [your command]

Debug this for me. What's wrong? How do I fix it?
Give me troubleshooting steps I can follow as a beginner.
```

**Expected outcome**: Specific debugging steps for your error with beginner-appropriate explanations.

